#!/usr/bin/env bash
set -euo pipefail

# Suppress undervoltage warnings (only if you trust your PSU)
CONFIG=/boot/firmware/config.txt
if ! grep -q '^avoid_warnings=2' "$CONFIG"; then
  echo 'avoid_warnings=2' | sudo tee -a "$CONFIG"
  echo "Undervoltage warnings suppressed (avoid_warnings=2)."
fi

# Base OS prep for Ubuntu 22.04 on Raspberry Pi 5
sudo apt update
sudo apt -y full-upgrade
sudo apt -y install build-essential curl git cmake pkg-config         python3-pip python3-venv python3-colcon-common-extensions         htop btop tmux unzip net-tools iperf3         libssl-dev libyaml-cpp-dev

# Enable zram swap (optional but helpful on SD cards)
if ! dpkg -l | grep -q zram-config; then
  sudo apt -y install zram-config || true
fi

echo "OS prep complete. Reboot recommended if kernel was upgraded."
