#!/usr/bin/env bash
#
# Prepare a freshly-flashed Ubuntu 24.04 arm64 SD card for the FollowBot Pi 5.
# Runs on macOS, against the mounted FAT boot partition — NOT on the Pi.
#
# Raspberry Pi Imager cannot customise Ubuntu images (its panel writes Pi OS's
# userconf.txt / wpa_supplicant.conf, which Ubuntu ignores). Ubuntu uses
# cloud-init's NoCloud datasource instead, reading user-data / meta-data /
# network-config from the boot partition. This writes those.
#
# Secrets are prompted for, never passed as arguments, and only ever written
# hashed: SHA-512 crypt for the account password, PBKDF2 for the WiFi PSK.
#
# Usage:  ./scripts/00_prepare_sd_macos.sh [--dry-run] [--boot /Volumes/system-boot]

# macOS ships bash 3.2; we need 4+ for ${var,,} and friends.
if (( BASH_VERSINFO[0] < 4 )); then
  for alt in /opt/homebrew/bin/bash /usr/local/bin/bash; do
    [[ -x $alt ]] && exec "$alt" "$0" "$@"
  done
  echo "This script needs bash 4+. Install with: brew install bash" >&2
  exit 1
fi

set -euo pipefail
umask 077

readonly MARKER_BEGIN="# >>> followbot:00_prepare_sd_macos >>>"
readonly MARKER_END="# <<< followbot:00_prepare_sd_macos <<<"
readonly UART_OVERLAY="uart2-pi5"   # GPIO 4/5, physical pins 7/29

DRY_RUN=0
BOOT=""

# --- output helpers ----------------------------------------------------------

if [[ -t 2 && -z ${NO_COLOR:-} ]]; then
  C_RESET=$'\033[0m'; C_BOLD=$'\033[1m'; C_DIM=$'\033[2m'
  C_RED=$'\033[31m'; C_GREEN=$'\033[32m'; C_YELLOW=$'\033[33m'; C_BLUE=$'\033[34m'
else
  C_RESET=""; C_BOLD=""; C_DIM=""; C_RED=""; C_GREEN=""; C_YELLOW=""; C_BLUE=""
fi

step() { printf '\n%s==>%s %s%s%s\n' "$C_BLUE" "$C_RESET" "$C_BOLD" "$*" "$C_RESET" >&2; }
info() { printf '    %s\n' "$*" >&2; }
ok()   { printf '  %s+%s %s\n' "$C_GREEN" "$C_RESET" "$*" >&2; }
warn() { printf '  %s!%s %s\n' "$C_YELLOW" "$C_RESET" "$*" >&2; }
die()  { printf '\n  %sx%s %s\n\n' "$C_RED" "$C_RESET" "$*" >&2; exit 1; }

# --- secret input ------------------------------------------------------------

# Read a secret twice from the terminal without echoing it.
#
# Reads from /dev/tty rather than stdin so a piped value can't slip in
# unnoticed (and land in shell history). xtrace is suspended for the duration
# so `bash -x` can never print what was typed.
#
#   read_secret PROMPT VARNAME [MIN_LEN] [MAX_LEN]
read_secret() {
  local prompt=$1 varname=$2 min=${3:-1} max=${4:-0}
  local first second attempts=0
  local xtrace_was_on=0
  case $- in *x*) xtrace_was_on=1; set +x ;; esac

  while true; do
    (( ++attempts > 3 )) && { (( xtrace_was_on )) && set -x; die "Too many failed attempts."; }

    printf '    %s: ' "$prompt" >&2
    IFS= read -rs first < /dev/tty || { (( xtrace_was_on )) && set -x; die "Could not read from terminal."; }
    printf '\n' >&2

    if [[ -z $first ]]; then
      warn "Must not be empty."
      continue
    fi
    if (( ${#first} < min )); then
      warn "Must be at least $min characters."
      continue
    fi
    if (( max > 0 && ${#first} > max )); then
      warn "Must be at most $max characters."
      continue
    fi

    printf '    %s (again): ' "$prompt" >&2
    IFS= read -rs second < /dev/tty
    printf '\n' >&2

    if [[ $first != "$second" ]]; then
      warn "Entries did not match."
      continue
    fi
    break
  done

  printf -v "$varname" '%s' "$first"
  unset first second
  (( xtrace_was_on )) && set -x
  return 0
}

# Suspend xtrace around any region that touches a plaintext secret, so
# `bash -x` can never print one. Paired: hide_trace ... restore_trace.
_XTRACE_DEPTH=0
hide_trace() {
  _XTRACE_WAS=0
  case $- in *x*) _XTRACE_WAS=1; set +x ;; esac
}
restore_trace() {
  (( ${_XTRACE_WAS:-0} )) && set -x
  return 0
}

ask() {  # ask PROMPT DEFAULT(y|n) -> 0 for yes
  local prompt=$1 default=${2:-y} reply
  local hint="[Y/n]"; [[ $default == n ]] && hint="[y/N]"
  printf '    %s %s ' "$prompt" "$hint" >&2
  IFS= read -r reply < /dev/tty || reply=""
  reply=${reply:-$default}
  [[ ${reply,,} == y* ]]
}

ask_value() {  # ask_value PROMPT DEFAULT VARNAME
  local prompt=$1 default=$2 varname=$3 reply
  if [[ -n $default ]]; then
    printf '    %s [%s]: ' "$prompt" "$default" >&2
  else
    printf '    %s: ' "$prompt" >&2
  fi
  IFS= read -r reply < /dev/tty || reply=""
  printf -v "$varname" '%s' "${reply:-$default}"
}

# --- preflight ---------------------------------------------------------------

OPENSSL=""
PYTHON=""

preflight_tools() {
  step "Checking tools"

  (( EUID == 0 )) && die "Do not run this as root. It writes to a FAT volume that needs no privileges."

  # macOS ships LibreSSL, which does NOT support -6 (SHA-512 crypt). Probe for
  # a real OpenSSL rather than trusting whatever `openssl` resolves to.
  local candidate
  for candidate in /opt/homebrew/bin/openssl /usr/local/bin/openssl "$(command -v openssl 2>/dev/null || true)"; do
    [[ -n $candidate && -x $candidate ]] || continue
    if printf 'x' | "$candidate" passwd -6 -stdin >/dev/null 2>&1; then
      OPENSSL=$candidate
      break
    fi
  done
  [[ -n $OPENSSL ]] || die "No OpenSSL with SHA-512 crypt support (-6).
      macOS ships LibreSSL, which lacks it. Install with:  brew install openssl@3"
  ok "openssl: $OPENSSL ($("$OPENSSL" version | cut -d' ' -f1-2))"

  for candidate in /opt/homebrew/bin/python3 /usr/local/bin/python3 "$(command -v python3 2>/dev/null || true)"; do
    [[ -n $candidate && -x $candidate ]] || continue
    if "$candidate" -c 'import hashlib; hashlib.pbkdf2_hmac("sha1", b"a", b"b", 1, 32)' >/dev/null 2>&1; then
      PYTHON=$candidate
      break
    fi
  done
  [[ -n $PYTHON ]] || die "No python3 with hashlib.pbkdf2_hmac (needed to derive the WiFi PSK)."
  ok "python3: $PYTHON"

  for tool in diskutil plutil stat ssh-keygen; do
    command -v "$tool" >/dev/null || die "Required tool not found: $tool"
  done
}

# --- boot partition discovery ------------------------------------------------

# Validate that a path is really an external FAT boot partition, and emphatically
# not the system volume. Checks are ordered cheapest-first.
validate_boot() {
  local path=$1 quiet=${2:-0}

  [[ -d $path ]] || { (( quiet )) || warn "Not a directory: $path"; return 1; }

  # Canonicalize before any prefix check, so symlinks can't smuggle us elsewhere.
  local real
  real=$(cd "$path" 2>/dev/null && pwd -P) || { (( quiet )) || warn "Cannot enter: $path"; return 1; }
  [[ $real == /Volumes/* ]] || { (( quiet )) || warn "Not under /Volumes: $real"; return 1; }
  [[ $(awk -F/ '{print NF}' <<< "$real") -eq 3 ]] || { (( quiet )) || warn "Not a volume root: $real"; return 1; }

  # The strongest guard: compare the resolved device id against the system
  # volumes. Survives renamed volumes, symlinks and firmlinks.
  local dev_target dev_root dev_data
  dev_target=$(stat -f '%d' "$real")
  dev_root=$(stat -f '%d' /)
  dev_data=$(stat -f '%d' /System/Volumes/Data 2>/dev/null || echo "-1")
  if [[ $dev_target == "$dev_root" || $dev_target == "$dev_data" ]]; then
    (( quiet )) || warn "REFUSING: $real is on the system volume."
    return 1
  fi

  local plist fstype ejectable removable
  plist=$(diskutil info -plist "$real" 2>/dev/null) || { (( quiet )) || warn "diskutil could not describe $real"; return 1; }
  fstype=$(plutil -extract FilesystemType raw - <<< "$plist" 2>/dev/null || echo "")
  ejectable=$(plutil -extract Ejectable raw - <<< "$plist" 2>/dev/null || echo "")
  removable=$(plutil -extract RemovableMedia raw - <<< "$plist" 2>/dev/null || echo "")

  if [[ $fstype == apfs || $fstype == hfs* ]]; then
    (( quiet )) || warn "REFUSING: $real is $fstype, not a FAT boot partition."
    return 1
  fi

  # Test removability, not Internal: a Mac's built-in SD slot sits on the
  # internal bus and reports Internal=true even though the card itself is
  # removable. Ejectable/RemovableMedia still separate real media from a
  # fixed disk's EFI partition, which is what this guard exists to reject.
  if [[ $ejectable != true && $removable != true ]]; then
    (( quiet )) || warn "REFUSING: $real is not on removable media (a fixed disk's EFI partition?)."
    return 1
  fi

  local size
  size=$(plutil -extract TotalSize raw - <<< "$plist" 2>/dev/null || echo 0)
  if (( size > 4294967296 )); then
    (( quiet )) || warn "REFUSING: $real is $((size / 1073741824)) GB — too large to be a boot partition."
    return 1
  fi

  [[ -f $real/config.txt && -f $real/cmdline.txt ]] || {
    (( quiet )) || warn "No config.txt + cmdline.txt in $real — not a Pi boot partition."
    return 1
  }

  printf '%s' "$real"
  return 0
}

find_boot_partition() {
  step "Locating the SD card boot partition"

  if [[ -n $BOOT ]]; then
    local resolved
    resolved=$(validate_boot "$BOOT") || die "The volume you passed with --boot did not pass validation."
    BOOT=$resolved
    ok "Using $BOOT"
    return
  fi

  local -a candidates=()
  local vol
  for vol in /Volumes/system-boot /Volumes/bootfs /Volumes/boot; do
    [[ -d $vol ]] && candidates+=("$vol")
  done
  if (( ${#candidates[@]} == 0 )); then
    for vol in /Volumes/*/; do
      vol=${vol%/}
      [[ -f $vol/config.txt && -f $vol/cmdline.txt ]] && candidates+=("$vol")
    done
  fi

  if (( ${#candidates[@]} == 0 )); then
    die "No SD card boot partition found under /Volumes.
      Insert the flashed card. If macOS did not mount it, run:
        diskutil list
        diskutil mount <identifier>
      (macOS cannot mount the ext4 root partition — only the FAT boot one. A
       \"disk not readable\" dialog for the other partition is expected.)"
  fi

  local chosen
  if (( ${#candidates[@]} == 1 )); then
    chosen=${candidates[0]}
  else
    info "Multiple candidates found:"
    local i
    for i in "${!candidates[@]}"; do
      printf '      %d) %s\n' "$((i + 1))" "${candidates[$i]}" >&2
    done
    local pick
    while true; do
      printf '    Which one? [1-%d]: ' "${#candidates[@]}" >&2
      IFS= read -r pick < /dev/tty || die "No selection."
      [[ $pick =~ ^[0-9]+$ ]] && (( pick >= 1 && pick <= ${#candidates[@]} )) && break
      warn "Enter a number between 1 and ${#candidates[@]}."
    done
    chosen=${candidates[$((pick - 1))]}
  fi

  local resolved
  resolved=$(validate_boot "$chosen") || die "$chosen failed validation; refusing to write to it."
  BOOT=$resolved
  ok "Found $BOOT"

  # Ubuntu ships all three cloud-init files; Pi OS ships none of them.
  if [[ ! -f $BOOT/user-data && ! -f $BOOT/network-config && ! -f $BOOT/meta-data ]]; then
    warn "This looks like Raspberry Pi OS, not Ubuntu — no cloud-init files present."
    warn "cloud-init files written here would be ignored by Pi OS."
    ask "Continue anyway?" n || die "Aborted."
  fi
}

confirm_target() {
  local plist name fstype size device
  plist=$(diskutil info -plist "$BOOT")
  name=$(plutil -extract VolumeName raw - <<< "$plist" 2>/dev/null || echo "?")
  fstype=$(plutil -extract FilesystemType raw - <<< "$plist" 2>/dev/null || echo "?")
  size=$(plutil -extract TotalSize raw - <<< "$plist" 2>/dev/null || echo 0)
  device=$(stat -f '%Sd' "$BOOT")

  step "Target volume"
  info "path     $BOOT"
  info "volume   $name"
  info "device   /dev/$device"
  info "type     $fstype"
  info "size     $((size / 1048576)) MB"

  (( DRY_RUN )) && { warn "Dry run — nothing will be written."; return; }

  printf '\n    %sType %syes%s to write to this volume:%s ' \
    "$C_BOLD" "$C_YELLOW" "$C_RESET$C_BOLD" "$C_RESET" >&2
  local reply
  IFS= read -r reply < /dev/tty || reply=""
  [[ $reply == yes ]] || die "Aborted — nothing was written."
}

# --- identity ----------------------------------------------------------------

HOSTNAME_VAL=""
USERNAME_VAL=""
PW_HASH=""

prompt_identity() {
  step "Identity"

  while true; do
    ask_value "Hostname" "followbot" HOSTNAME_VAL
    HOSTNAME_VAL=${HOSTNAME_VAL,,}
    [[ $HOSTNAME_VAL =~ ^[a-z0-9]([a-z0-9-]{0,61}[a-z0-9])?$ ]] && break
    warn "Invalid hostname. Use letters, digits and hyphens; must not start or end with a hyphen."
  done

  while true; do
    ask_value "Username" "${USER:-pi}" USERNAME_VAL
    USERNAME_VAL=${USERNAME_VAL,,}
    if [[ ! $USERNAME_VAL =~ ^[a-z_][a-z0-9_-]{0,31}$ ]]; then
      warn "Invalid username. Start with a letter or underscore; letters, digits, hyphen, underscore only."
      continue
    fi
    case $USERNAME_VAL in
      root|daemon|bin|sys|sync|games|man|lp|mail|news|uucp|proxy|www-data|backup|nobody)
        warn "'$USERNAME_VAL' is a reserved system account."; continue ;;
    esac
    if [[ $USERNAME_VAL == ubuntu ]]; then
      warn "'ubuntu' collides with the image's default user; cloud-init will reconfigure it."
      ask "Use it anyway?" n || continue
    fi
    break
  done
}

prompt_password() {
  step "Account password"
  info "Used for console and sudo login. SSH will use your key, not this."

  local password
  read_secret "Password for $USERNAME_VAL" password 1

  if (( ${#password} < 8 )); then
    warn "That password is under 8 characters. The hash on the card is offline-crackable."
    if ! ask "Use it anyway?" n; then
      unset password
      prompt_password
      return
    fi
  fi

  # -stdin keeps the password out of argv, where `ps` could see it; hide_trace
  # keeps it out of `bash -x` output.
  hide_trace
  PW_HASH=$(printf '%s' "$password" | "$OPENSSL" passwd -6 -stdin) \
    || { restore_trace; die "Password hashing failed."; }
  unset password
  restore_trace

  [[ $PW_HASH == \$6\$* ]] || die "Unexpected hash format: expected SHA-512 crypt."
  ok "Hashed with SHA-512 crypt."
}

# --- ssh keys ----------------------------------------------------------------

SSH_KEY=""
SSH_PWAUTH="false"

# Collect public keys from every agent we can find, then from disk.
#
# ssh-add only ever queries $SSH_AUTH_SOCK and never parses ssh_config, so an
# IdentityAgent directive (1Password, Secretive, gpg-agent) is invisible to it.
# We parse those out and query each socket directly. Keys held in such an agent
# usually have no .pub file at all, so a disk-only search would miss them.
collect_ssh_keys() {
  local -n _lines=$1 _labels=$2
  local -A seen=()

  add_key() {  # add_key "<pubkey line>" "<source label>"
    local line=$1 source=$2 blob
    line=${line%%$'\r'}
    [[ $line =~ ^(ssh-ed25519|ssh-rsa|ssh-dss|ecdsa-sha2-[a-z0-9-]+|sk-ssh-ed25519@openssh\.com|sk-ecdsa-sha2-[a-z0-9-]+@openssh\.com)[[:space:]] ]] || return 0
    blob=$(awk '{print $2}' <<< "$line")
    [[ -n $blob && -z ${seen[$blob]:-} ]] || return 0
    seen[$blob]=1
    _lines+=("$line")
    _labels+=("$source")
  }

  # 1. IdentityAgent sockets declared in ssh_config.
  local -a sockets=()
  if [[ -f ~/.ssh/config ]]; then
    while IFS= read -r sock; do
      sock=${sock%\"}; sock=${sock#\"}
      sock=${sock/#\~/$HOME}
      [[ $sock == SSH_AUTH_SOCK ]] && sock=${SSH_AUTH_SOCK:-}
      [[ -n $sock && -S $sock ]] && sockets+=("$sock")
    done < <(awk 'tolower($1)=="identityagent" {$1=""; sub(/^[ \t]+/,""); print}' ~/.ssh/config)
  fi
  [[ -n ${SSH_AUTH_SOCK:-} && -S ${SSH_AUTH_SOCK:-} ]] && sockets+=("$SSH_AUTH_SOCK")

  local sock seen_socks=""
  for sock in "${sockets[@]:-}"; do
    [[ -n $sock ]] || continue
    [[ $seen_socks == *"|$sock|"* ]] && continue
    seen_socks+="|$sock|"

    local label="ssh agent"
    [[ $sock == *1password* ]] && label="1Password"
    [[ $sock == *secretive* ]] && label="Secretive"

    local agent_out
    # Listing public keys needs no private-key operation, so this should not
    # prompt for an unlock. Redirect stdin from /dev/null so a stuck agent
    # cannot block on input.
    agent_out=$(SSH_AUTH_SOCK="$sock" ssh-add -L 2>/dev/null </dev/null || true)
    [[ -n $agent_out && $agent_out != *"no identities"* ]] || continue
    while IFS= read -r line; do
      [[ -n $line ]] && add_key "$line" "$label"
    done <<< "$agent_out"
  done

  # 2. Public keys on disk.
  local pub
  for pub in "$HOME"/.ssh/*.pub; do
    [[ -f $pub ]] || continue
    add_key "$(< "$pub")" "${pub/#$HOME/~}"
  done

  unset -f add_key
}

prompt_ssh_key() {
  step "SSH key"

  local -a key_lines=() key_labels=()
  collect_ssh_keys key_lines key_labels

  if (( ${#key_lines[@]} == 0 )); then
    warn "No SSH public keys found in any agent or in ~/.ssh."
  else
    local i fp type comment
    for i in "${!key_lines[@]}"; do
      fp=$(ssh-keygen -lf - <<< "${key_lines[$i]}" 2>/dev/null) || fp=""
      type=$(awk '{print $1}' <<< "${key_lines[$i]}" | sed 's/^ssh-//; s/@openssh\.com$//')
      comment=$(cut -d' ' -f3- <<< "${key_lines[$i]}")
      [[ -z $comment ]] && comment="(no comment)"
      printf '      %2d) %-10s %-22s %-28s %s[%s]%s\n' \
        "$((i + 1))" "$type" "$(awk '{print $2}' <<< "$fp" | cut -c1-20)…" \
        "$comment" "$C_DIM" "${key_labels[$i]}" "$C_RESET" >&2
    done
    printf '       p) paste a public key\n' >&2
    printf '       s) skip (enables SSH password authentication)\n' >&2
  fi

  local pick
  while true; do
    printf '    Which key? ' >&2
    IFS= read -r pick < /dev/tty || pick="s"

    if [[ $pick == p ]]; then
      printf '    Paste the public key line: ' >&2
      IFS= read -r SSH_KEY < /dev/tty
      if ssh-keygen -lf - <<< "$SSH_KEY" >/dev/null 2>&1; then
        ok "Key accepted."
        return
      fi
      warn "That did not parse as a public key."
      continue
    fi

    if [[ $pick == s ]]; then
      warn "No SSH key — enabling password authentication so the Pi stays reachable."
      SSH_KEY=""
      SSH_PWAUTH="true"
      return
    fi

    if [[ $pick =~ ^[0-9]+$ ]] && (( pick >= 1 && pick <= ${#key_lines[@]} )); then
      SSH_KEY=${key_lines[$((pick - 1))]}
      ok "Using ${key_labels[$((pick - 1))]} key: $(cut -d' ' -f3- <<< "$SSH_KEY")"
      return
    fi

    warn "Enter a number, 'p' to paste, or 's' to skip."
  done
}

# --- wifi --------------------------------------------------------------------

WIFI_SSID=""
WIFI_SECRET=""       # what actually goes in the file (derived PSK or passphrase)
WIFI_IS_SAE=0
WIFI_HIDDEN=0
WIFI_COUNTRY=""

prompt_wifi() {
  step "WiFi"

  if ! ask "Configure WiFi?" y; then
    WIFI_SSID=""
    return
  fi

  while true; do
    ask_value "SSID" "" WIFI_SSID
    [[ -n $WIFI_SSID ]] && (( ${#WIFI_SSID} <= 32 )) && break
    warn "SSID must be 1-32 characters."
  done

  if ask "Is this a hidden network?" n; then WIFI_HIDDEN=1; else WIFI_HIDDEN=0; fi

  info ""
  info "Security:"
  info "  1) WPA3 Personal, or WPA2/WPA3 mixed — passphrase stored as-is"
  info "  2) WPA2 only — PSK is derived; the passphrase never lands on the card"
  info ""
  info "    SAE (WPA3) derives its key from the passphrase during each handshake,"
  info "    so a pre-derived PSK cannot be used. Choose 2 only if the network is"
  info "    WPA2-only; a WPA3 or mixed AP needs 1."
  local mode
  while true; do
    ask_value "Choice" "1" mode
    [[ $mode == 1 || $mode == 2 ]] && break
    warn "Enter 1 or 2."
  done
  [[ $mode == 1 ]] && WIFI_IS_SAE=1

  while true; do
    ask_value "WiFi country code (2 letters, sets the regulatory domain)" "US" WIFI_COUNTRY
    WIFI_COUNTRY=${WIFI_COUNTRY^^}
    [[ $WIFI_COUNTRY =~ ^[A-Z]{2}$ ]] && break
    warn "Use a two-letter code such as US, GB, DE."
  done

  local psk
  read_secret "WiFi passphrase" psk 8 63

  # Everything from here until the secret is unset stays out of xtrace.
  hide_trace

  # An already-derived 64-hex PSK passes straight through — but never for SAE.
  # netplan happily writes a hex psk= for a WPA3 network and `netplan generate`
  # succeeds, yet wpa_supplicant nulls the passphrase when it sees unquoted hex
  # and SAE reads only the passphrase. The result is a silent failure to join.
  if [[ $psk =~ ^[0-9a-fA-F]{64}$ ]]; then
    if (( WIFI_IS_SAE )); then
      unset psk
      restore_trace
      die "A 64-hex PSK cannot be used with WPA3/SAE — it needs the real passphrase.
      Re-run and enter the passphrase itself, or choose WPA2-only if that is
      what the network actually runs."
    fi
    WIFI_SECRET=${psk,,}
    unset psk
    restore_trace
    ok "Recognised a pre-derived 64-hex PSK."
    return
  fi

  # PBKDF2 needs the SSID as a byte-exact salt. If the SSID is not plain ASCII
  # we cannot be sure our bytes match what the AP broadcasts, and a wrong PMK
  # fails silently at join time — so store the passphrase instead.
  # A VAR=val prefix does not apply to shell builtins, so set LC_ALL for the
  # whole comparison instead of prefixing [[ ]].
  local ascii_ssid=1 saved_lc=${LC_ALL:-}
  LC_ALL=C
  [[ $WIFI_SSID =~ ^[[:print:]]+$ ]] || ascii_ssid=0
  LC_ALL=$saved_lc

  if (( WIFI_IS_SAE )); then
    warn "WPA3-SAE: storing the passphrase in plaintext (SAE cannot use a derived PSK)."
    WIFI_SECRET=$psk
  elif (( ! ascii_ssid )); then
    warn "Non-ASCII SSID: storing the passphrase in plaintext (PBKDF2 salt would be ambiguous)."
    WIFI_SECRET=$psk
  else
    # Values go through the environment, never argv, so `ps` cannot see them.
    WIFI_SECRET=$(FB_SSID="$WIFI_SSID" FB_PSK="$psk" "$PYTHON" -c '
import binascii, hashlib, os
print(binascii.hexlify(hashlib.pbkdf2_hmac(
    "sha1", os.environb[b"FB_PSK"], os.environb[b"FB_SSID"], 4096, 32)).decode())
') || { restore_trace; die "PSK derivation failed."; }
    [[ $WIFI_SECRET =~ ^[0-9a-f]{64}$ ]] || { restore_trace; die "PSK derivation produced unexpected output."; }
    unset psk
    restore_trace
    ok "Derived a 256-bit PSK; the passphrase will not be written to the card."
    return
  fi
  unset psk
  restore_trace
}

# --- hardware overlays -------------------------------------------------------

WANT_UART=1
WANT_CAMERAS=0
WANT_PCIE=0

prompt_hardware() {
  step "Hardware overlays (config.txt)"

  info "The RVR needs a free UART. On a Pi 5 we use $UART_OVERLAY (GPIO 4/5,"
  info "physical pins 7 and 29), which avoids Bluetooth and the debug console."
  # Plain if/else, not `ask ... && VAR=1`: under `set -e` a trailing && whose
  # left side is false makes the function return non-zero and aborts the script.
  if ask "Enable UART for the Sphero RVR?" y; then WANT_UART=1; else WANT_UART=0; fi
  if ask "Enable two IMX708 CSI cameras (Arducam B0309, cam0 + cam1)?" n; then WANT_CAMERAS=1; else WANT_CAMERAS=0; fi
  if ask "Enable PCIe gen 3 for the Hailo-8L AI HAT+?" n; then WANT_PCIE=1; else WANT_PCIE=0; fi
}

# --- rendering ---------------------------------------------------------------

# YAML single-quoted scalar: one rule that survives spaces, #, :, and digits.
yaml_squote() { printf "'%s'" "${1//\'/\'\'}"; }

render_user_data() {
  local ssh_block=""
  if [[ -n $SSH_KEY ]]; then
    ssh_block="    ssh_authorized_keys:
      - $SSH_KEY"
  fi

  local udev_block=""
  if (( WANT_UART )); then
    udev_block="
write_files:
  # The uartN -> ttyAMAn mapping is not officially documented for the Pi 5, so
  # pin a stable name here instead of hardcoding a number in the robot code.
  # Verify with: ls -l /dev/serial* /dev/ttyAMA*
  - path: /etc/udev/rules.d/99-followbot.rules
    permissions: '0644'
    content: |
      SUBSYSTEM==\"tty\", KERNEL==\"ttyAMA2\", SYMLINK+=\"rvr\", GROUP=\"dialout\", MODE=\"0660\"
"
  fi

  cat <<EOF
#cloud-config
# Written by scripts/00_prepare_sd_macos.sh on $(date -u '+%Y-%m-%dT%H:%M:%SZ')
# Target: Raspberry Pi 5, Ubuntu 24.04 arm64

hostname: $HOSTNAME_VAL
manage_etc_hosts: true

users:
  - name: $USERNAME_VAL
    gecos: FollowBot operator
    shell: /bin/bash
    sudo: "ALL=(ALL) NOPASSWD:ALL"
    # Cloud-init locks the account by default even when passwd: is set.
    lock_passwd: false
    passwd: "$PW_HASH"
    groups: [adm, sudo, dialout, video, audio, plugdev, netdev, input, render]
$ssh_block

ssh_pwauth: $SSH_PWAUTH
disable_root: true

package_update: true
package_upgrade: false
packages:
  - avahi-daemon
  - openssh-server
$udev_block
runcmd:
  # i2c and gpio may not exist on a fresh image, and an unknown group in the
  # users: block can fail user creation outright — so add them here instead.
  - [ groupadd, -f, i2c ]
  - [ groupadd, -f, gpio ]
  - [ usermod, -aG, "i2c,gpio", $USERNAME_VAL ]
  # Ubuntu Desktop otherwise runs a first-boot wizard on the attached display.
  - [ mkdir, -p, /home/$USERNAME_VAL/.config ]
  - [ touch, /home/$USERNAME_VAL/.config/gnome-initial-setup-done ]
  - [ chown, -R, "$USERNAME_VAL:$USERNAME_VAL", /home/$USERNAME_VAL/.config ]
  - [ systemctl, mask, gnome-initial-setup-first-login.service ]
  - [ systemctl, enable, --now, ssh ]
EOF
}

render_network_config() {
  if [[ -z $WIFI_SSID ]]; then
    cat <<'EOF'
version: 2
ethernets:
  eth0:
    dhcp4: true
    optional: true
EOF
    return
  fi

  # No renderer: key — 24.04 Desktop defaults to NetworkManager, and forcing
  # networkd yields a connection the desktop will not manage.
  cat <<EOF
version: 2
ethernets:
  eth0:
    dhcp4: true
    optional: true
wifis:
  wlan0:
    dhcp4: true
    optional: true
    regulatory-domain: "$WIFI_COUNTRY"
    access-points:
      $(yaml_squote "$WIFI_SSID"):
EOF
  (( WIFI_HIDDEN )) && printf '        hidden: true\n'
  # The bare `password:` form is deliberate for WPA3 too. Since netplan 1.0
  # (which noble ships) it emits key_mgmt=WPA-PSK WPA-PSK-SHA256 SAE with PMF
  # optional, so it negotiates WPA2 or WPA3 with whatever the AP offers. An
  # explicit `auth: {key-management: sae}` block would emit key_mgmt=SAE alone
  # with PMF mandatory — narrower, and it cannot fall back if the Pi 5's
  # brcmfmac firmware turns out to lack SAE support.
  printf '        password: "%s"\n' "$WIFI_SECRET"
}

render_config_block() {
  printf '%s\n' "$MARKER_BEGIN"
  # [all] resets any preceding conditional filter ([pi4], [cm5], ...) so these
  # lines apply regardless of where the block lands in the file.
  printf '[all]\n'
  if (( WANT_UART )); then
    printf '# Sphero RVR on GPIO 4/5 (physical pins 7 and 29).\n'
    printf '# Not enable_uart=1 — on a Pi 5 that can route kernel logs to GPIO 14/15.\n'
    printf 'dtoverlay=%s\n' "$UART_OVERLAY"
  fi
  if (( WANT_CAMERAS )); then
    printf '# Dual Arducam B0309 (IMX708).\n'
    printf 'camera_auto_detect=0\n'
    printf 'dtoverlay=imx708,cam0\n'
    printf 'dtoverlay=imx708,cam1\n'
  fi
  if (( WANT_PCIE )); then
    printf '# Hailo-8L AI HAT+ on the M.2 slot.\n'
    printf 'dtparam=pciex1\n'
    printf 'dtparam=pciex1_gen=3\n'
  fi
  printf '%s\n' "$MARKER_END"
}

# --- writing -----------------------------------------------------------------

backup_file() {
  local path=$1
  [[ -f $path ]] || return 0
  [[ -f $path.orig ]] || cp -p "$path" "$path.orig"
  cp -p "$path" "$path.followbot-bak.$(date '+%Y%m%dT%H%M%S')"
}

write_atomic() {  # content on stdin
  local dest=$1 tmp
  tmp=$(mktemp "$dest.tmp.XXXXXX") || die "Could not create a temp file next to $dest"
  cat > "$tmp"
  # Never commit an empty result: a failed producer upstream would otherwise
  # truncate the destination, which for config.txt means an unbootable Pi.
  if [[ ! -s $tmp ]]; then
    rm -f "$tmp"
    die "Refusing to write an empty $dest (the content producer failed)."
  fi
  mv -f "$tmp" "$dest"
}

# Replace our marker block if present, otherwise append it. Replacing rather
# than appending is what makes re-runs idempotent.
patch_config_txt() {
  local path=$BOOT/config.txt
  local block
  block=$(render_config_block)

  local has_begin=0 has_end=0
  grep -qF "$MARKER_BEGIN" "$path" && has_begin=1
  grep -qF "$MARKER_END" "$path" && has_end=1

  if (( has_begin != has_end )); then
    die "config.txt has an unmatched FollowBot marker — a previous run was interrupted.
      Inspect it, or restore from $path.orig, then re-run."
  fi

  backup_file "$path"

  if (( has_begin )); then
    # The block is multi-line, and BSD awk rejects newlines in -v values, so
    # hand it over as a file rather than a variable.
    local blockfile
    blockfile=$(mktemp "${TMPDIR:-/tmp}/followbot-block.XXXXXX")
    printf '%s\n' "$block" > "$blockfile"
    awk -v begin="$MARKER_BEGIN" -v end="$MARKER_END" -v blockfile="$blockfile" '
      index($0, begin) { while ((getline line < blockfile) > 0) print line; skip = 1; next }
      index($0, end)   { skip = 0; next }
      !skip            { print }
    ' "$path" > "$blockfile.out" || { rm -f "$blockfile" "$blockfile.out"; die "Rewriting config.txt failed."; }
    write_atomic "$path" < "$blockfile.out"
    rm -f "$blockfile" "$blockfile.out"
    ok "config.txt: replaced the existing FollowBot block"
  else
    local body
    body=$(cat "$path")
    { printf '%s\n\n%s\n' "$body" "$block"; } | write_atomic "$path"
    ok "config.txt: appended the FollowBot block"
  fi
}

write_all() {
  step "Writing to $BOOT"

  backup_file "$BOOT/user-data"
  render_user_data | write_atomic "$BOOT/user-data"
  ok "user-data"

  backup_file "$BOOT/network-config"
  render_network_config | write_atomic "$BOOT/network-config"
  ok "network-config"

  # Always rewrite meta-data with a fresh instance-id. Ubuntu's image ships
  # `instance_id: cloud-image` — a fixed string identical on every flash — and
  # cloud-init only runs per-instance modules (user creation, SSH keys,
  # hostname) when the instance-id CHANGES from the cached one. Preserving the
  # stock value means a card that has booted before silently skips user-data.
  backup_file "$BOOT/meta-data"
  # dsmode: local matches what Ubuntu's image ships — it applies user-data
  # before networking comes up, which is what a local FAT seed wants.
  printf 'dsmode: local\ninstance-id: followbot-%s\nlocal-hostname: %s\n' \
    "$(date '+%Y%m%d%H%M%S')" "$HOSTNAME_VAL" | write_atomic "$BOOT/meta-data"
  ok "meta-data (fresh instance-id, forces re-provisioning)"

  if (( WANT_UART || WANT_CAMERAS || WANT_PCIE )); then
    patch_config_txt
  else
    info "config.txt: nothing to change"
  fi

  # cmdline.txt is deliberately untouched: on a Pi 5, console=serial0 maps to
  # ttyAMA10 (the dedicated debug header), which does not collide with uart2.

  sync
}

verify_written() {
  local errors=0
  [[ $(head -1 "$BOOT/user-data") == "#cloud-config" ]] || { warn "user-data is missing its #cloud-config header"; errors=1; }
  [[ $(grep -c . "$BOOT/cmdline.txt") -le 1 ]] || { warn "cmdline.txt has more than one line"; errors=1; }
  if (( WANT_UART || WANT_CAMERAS || WANT_PCIE )); then
    [[ $(grep -cF "$MARKER_BEGIN" "$BOOT/config.txt") -eq 1 ]] || { warn "config.txt marker count is wrong"; errors=1; }
  fi
  if command -v ruby >/dev/null; then
    # aliases: true — Psych 4+ rejects YAML anchors by default.
    ruby -ryaml -e 'YAML.load_file(ARGV[0], aliases: true)' "$BOOT/network-config" 2>/dev/null \
      || { warn "network-config is not valid YAML"; errors=1; }
  fi
  (( errors == 0 )) && ok "Verified written files"
}

print_summary() {
  local device
  device=$(stat -f '%Sd' "$BOOT")

  cat >&2 <<EOF

${C_GREEN}${C_BOLD}SD card ready${C_RESET}  $BOOT (/dev/$device)

  ${C_BOLD}Account${C_RESET}
    hostname   $HOSTNAME_VAL
    user       $USERNAME_VAL (sudo NOPASSWD; groups include dialout, video, render)
    password   SHA-512 crypt hash written
    ssh        $(if [[ -n $SSH_KEY ]]; then cut -d' ' -f3- <<< "$SSH_KEY"; else echo "no key — password auth ENABLED"; fi)

  ${C_BOLD}Network${C_RESET}
$(if [[ -n $WIFI_SSID ]]; then
    printf '    wlan0      %s (%s), country %s\n' "$WIFI_SSID" \
      "$(if (( WIFI_IS_SAE )); then echo "WPA3-SAE, passphrase stored"; else echo "WPA2, PSK derived"; fi)" "$WIFI_COUNTRY"
  else
    printf '    wlan0      not configured\n'
  fi)
    eth0       DHCP, optional

  ${C_BOLD}Hardware${C_RESET}
$( (( WANT_UART ))    && printf '    UART       dtoverlay=%s -> /dev/rvr (GPIO 4/5, pins 7 and 29)\n' "$UART_OVERLAY" )
$( (( WANT_CAMERAS )) && printf '    Cameras    dual IMX708 on cam0 and cam1\n' )
$( (( WANT_PCIE ))    && printf '    PCIe       gen 3 for the Hailo-8L\n' )

  ${C_BOLD}Next${C_RESET}
    1.  diskutil eject $BOOT
    2.  Boot the Pi. First boot takes 2-4 minutes while cloud-init runs.
    3.  ssh $USERNAME_VAL@$HOSTNAME_VAL.local
    4.  Confirm the UART node, then fix the udev rule if it differs:
          ls -l /dev/serial* /dev/ttyAMA*   # expecting ttyAMA2 -> /dev/rvr
    5.  Run the on-Pi scripts starting with scripts/01_os_prep.sh

  ${C_YELLOW}Note${C_RESET}
    Everything on this FAT partition is readable by anyone holding the card.
    It contains a password hash and a derived WiFi key — not plaintext, but
    both are crackable offline. Use a strong password.

    Re-running this script only affects a card that has never booted:
    cloud-init re-applies per-instance modules only when instance-id changes.

EOF
}

# --- main --------------------------------------------------------------------

usage() {
  cat <<'EOF'
Prepare a flashed Ubuntu 24.04 SD card for the FollowBot Pi 5.

  --boot PATH   Use this boot partition instead of searching /Volumes
  --dry-run     Render everything and print it (secrets redacted); write nothing
  -h, --help    Show this help

Secrets are always prompted for. There is deliberately no flag to pass a
password or WiFi passphrase — that would put them in your shell history.
EOF
}

main() {
  while (( $# )); do
    case $1 in
      --boot)    BOOT=${2:-}; shift 2 || die "--boot needs a path" ;;
      --dry-run) DRY_RUN=1; shift ;;
      -h|--help) usage; exit 0 ;;
      *)         die "Unknown option: $1 (try --help)" ;;
    esac
  done

  printf '%s%sFollowBot SD card setup%s — Ubuntu 24.04 arm64 on Raspberry Pi 5\n' \
    "$C_BOLD" "$C_BLUE" "$C_RESET" >&2

  preflight_tools
  find_boot_partition
  confirm_target
  prompt_identity
  prompt_password
  prompt_ssh_key
  prompt_wifi
  prompt_hardware

  if (( DRY_RUN )); then
    step "Dry run — user-data"
    render_user_data | sed 's/^\(    passwd: "\).*/\1$6$***REDACTED***"/' >&2
    step "Dry run — network-config"
    render_network_config | sed 's/^\( *password: "\).*/\1***REDACTED***"/' >&2
    step "Dry run — config.txt block"
    render_config_block >&2
    printf '\n' >&2
    warn "Nothing was written."
    exit 0
  fi

  write_all
  verify_written
  print_summary
}

main "$@"
