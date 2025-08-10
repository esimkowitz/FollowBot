#!/usr/bin/env bash
set -euo pipefail
# Install Rust toolchain and scaffold ros2-rust build prereqs.
# Note: ros2-rust is built from source; verify the repo for latest instructions.

# Rust toolchain
if ! command -v rustup >/dev/null 2>&1; then
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
  source $HOME/.cargo/env
fi
rustup default stable
rustup component add rustfmt clippy

# Helpful system deps
sudo apt -y install libclang-dev

cat <<'EOF'
------------------------------------------------------------
NEXT STEPS (ros2-rust):
  1) Clone ros2-rust super-repo into a parallel ws:
     git clone https://github.com/ros2-rust/ros2_rust.git ~/ros2_rust
  2) Build:
     cd ~/ros2_rust
     source /opt/ros/humble/setup.bash
     colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
     source install/setup.bash
  3) In this workspace, uncomment Rust message/action uses and rebuild.
------------------------------------------------------------
EOF
