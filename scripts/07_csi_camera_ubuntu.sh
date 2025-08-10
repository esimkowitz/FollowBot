#!/usr/bin/env bash
set -euo pipefail
# Enable CSI camera on Ubuntu 22.04 (Pi 5): configure overlays, build libcamera + rpicam-apps.

CONFIG=/boot/firmware/config.txt
SENSOR_OVERLAY="${1:-imx708}"   # imx219 (V2), imx477 (HQ), imx708 (Cam Module 3)

sudo sed -i 's/^camera_auto_detect=.*/camera_auto_detect=1/' "$CONFIG" || true
if ! grep -q '^camera_auto_detect' "$CONFIG"; then
  echo 'camera_auto_detect=1' | sudo tee -a "$CONFIG"
fi
echo "Set camera_auto_detect=1 in $CONFIG. Reboot and test first."

read -p "If autodetect fails, press Enter to force overlay ($SENSOR_OVERLAY)..." _
sudo sed -i 's/^camera_auto_detect=.*/camera_auto_detect=0/' "$CONFIG" || true
if grep -q '^dtoverlay=imx' "$CONFIG"; then
  sudo sed -i 's/^dtoverlay=imx.*/dtoverlay='"$SENSOR_OVERLAY"'/' "$CONFIG"
else
  echo "dtoverlay=$SENSOR_OVERLAY" | sudo tee -a "$CONFIG"
fi
echo "Overlay set to $SENSOR_OVERLAY. Reboot recommended."

# Build libcamera (RPi fork) + rpicam-apps
sudo apt -y install git meson ninja-build cmake pkg-config       libgnutls28-dev openssl libtiff-dev libjpeg-dev libpng-dev       libdrm-dev libboost-dev libyaml-dev python3-yaml       libglib2.0-dev libgstreamer-plugins-base1.0-dev libfmt-dev       libavcodec-dev libavformat-dev libswresample-dev libswscale-dev

mkdir -p ~/camera_src && cd ~/camera_src
if [ ! -d libcamera ]; then
  git clone https://github.com/raspberrypi/libcamera.git
fi
cd libcamera
meson setup build --buildtype=release       -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp       -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled       -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
ninja -C build
sudo ninja -C build install
sudo ldconfig

cd ~/camera_src
if [ ! -d rpicam-apps ]; then
  git clone https://github.com/raspberrypi/rpicam-apps.git
fi
cd rpicam-apps
meson setup build --buildtype=release
ninja -C build
sudo ninja -C build install

echo "CSI camera stack installed. Test with: rpicam-hello --timeout 3000"
