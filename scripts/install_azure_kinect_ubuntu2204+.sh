#!/bin/bash
#
# Azure Kinect SDK Installation Script for Ubuntu 22.04 / 24.04
#
# Microsoft's official packages only support Ubuntu 18.04, but the packages
# work on newer Ubuntu versions with some manual steps.
#
# Usage: sudo ./install_azure_kinect_ubuntu2204+.sh
#
# For WSL2 users: You need to pass through the Azure Kinect USB devices using usbipd-win
#   In PowerShell (Admin):
#     usbipd bind --force --busid <BUSID>   # for both camera devices
#     usbipd attach --wsl --busid <BUSID>
#
# Author: GPS-SLAM Team
# Date: 2025
#

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
echo_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
echo_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo_error "Please run as root: sudo $0"
    exit 1
fi

# Check Ubuntu version
. /etc/os-release
echo_info "Detected: $PRETTY_NAME"

if [[ "$VERSION_ID" != "22.04" && "$VERSION_ID" != "24.04" ]]; then
    echo_warn "This script is tested on Ubuntu 22.04 and 24.04"
    echo_warn "Your version ($VERSION_ID) may work but is untested"
    read -p "Continue anyway? [y/N] " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Create temp directory
TEMP_DIR=$(mktemp -d)
cd "$TEMP_DIR"
echo_info "Working in $TEMP_DIR"

# Install base dependencies
echo_info "Installing base dependencies..."
apt-get update
apt-get install -y wget libusb-1.0-0 libssl3 libudev1

# Handle libsoundio dependency
echo_info "Installing libsoundio..."
if ! apt-get install -y libsoundio2 2>/dev/null; then
    echo_warn "libsoundio2 not in repos, downloading manually..."
    # Try different Ubuntu versions for the package
    if wget -q http://archive.ubuntu.com/ubuntu/pool/universe/libs/libsoundio/libsoundio2_2.0.0-2_amd64.deb 2>/dev/null; then
        dpkg -i libsoundio2_2.0.0-2_amd64.deb || true
    elif wget -q http://archive.ubuntu.com/ubuntu/pool/universe/libs/libsoundio/libsoundio1_1.1.0-1_amd64.deb 2>/dev/null; then
        dpkg -i libsoundio1_1.1.0-1_amd64.deb || true
    else
        echo_warn "Could not install libsoundio - k4aviewer may not work but SDK should be fine"
    fi
fi

# Download Azure Kinect SDK packages from Microsoft's Ubuntu 18.04 repo
echo_info "Downloading Azure Kinect SDK packages..."
K4A_VERSION="1.4.1"
BASE_URL="https://packages.microsoft.com/ubuntu/18.04/prod/pool/main"

wget -q --show-progress "${BASE_URL}/libk/libk4a${K4A_VERSION}/libk4a${K4A_VERSION}_${K4A_VERSION}_amd64.deb"
wget -q --show-progress "${BASE_URL}/libk/libk4a${K4A_VERSION}-dev/libk4a${K4A_VERSION}-dev_${K4A_VERSION}_amd64.deb"
wget -q --show-progress "${BASE_URL}/k/k4a-tools/k4a-tools_${K4A_VERSION}_amd64.deb"

# Install the packages
echo_info "Installing Azure Kinect SDK..."
dpkg -i "libk4a${K4A_VERSION}_${K4A_VERSION}_amd64.deb" || true
dpkg -i "libk4a${K4A_VERSION}-dev_${K4A_VERSION}_amd64.deb" || true
dpkg -i "k4a-tools_${K4A_VERSION}_amd64.deb" || true

# Fix any broken dependencies
apt-get --fix-broken install -y

# Create depth engine symlink (required for SDK 1.4)
echo_info "Creating depth engine symlink..."
LIBDIR="/usr/lib/x86_64-linux-gnu"
if [ -f "${LIBDIR}/libk4a${K4A_VERSION}/libdepthengine.so.2.0" ]; then
    ln -sf "${LIBDIR}/libk4a${K4A_VERSION}/libdepthengine.so.2.0" "${LIBDIR}/libdepthengine.so"
    echo_info "Symlink created: ${LIBDIR}/libdepthengine.so"
else
    echo_warn "Depth engine library not found - depth processing may not work"
fi

# Set up udev rules for non-root access
echo_info "Setting up udev rules..."
cat > /etc/udev/rules.d/99-k4a.rules << 'EOF'
# Bus 002 Device 116: ID 045e:097a Microsoft Corp.  - Generic Superspeed USB Hub
# Bus 001 Device 015: ID 045e:097b Microsoft Corp.  - Generic USB Hub
# Bus 002 Device 118: ID 045e:097c Microsoft Corp.  - Azure Kinect Depth Camera
# Bus 002 Device 117: ID 045e:097d Microsoft Corp.  - Azure Kinect 4K Camera
# Bus 001 Device 016: ID 045e:097e Microsoft Corp.  - Azure Kinect Microphone Array

BUS!="usb", ACTION!="add", SUBSYSTEM!=="usb_device", GOTO="k4a_logic_rules_end"

ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097a", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097b", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097c", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097d", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097e", MODE="0666", GROUP="plugdev"

LABEL="k4a_logic_rules_end"
EOF

# Reload udev rules
udevadm control --reload-rules
udevadm trigger

echo_info "Adding current user to plugdev group..."
SUDO_USER_ACTUAL="${SUDO_USER:-$USER}"
usermod -aG plugdev "$SUDO_USER_ACTUAL" 2>/dev/null || true

# Cleanup
echo_info "Cleaning up..."
cd /
rm -rf "$TEMP_DIR"

# Verify installation
echo ""
echo_info "=== Installation Complete ==="
echo ""

if command -v k4aviewer &> /dev/null; then
    echo_info "k4aviewer installed: $(which k4aviewer)"
else
    echo_warn "k4aviewer not found in PATH"
fi

if [ -f "${LIBDIR}/libk4a.so" ] || [ -f "${LIBDIR}/libk4a.so.${K4A_VERSION}" ]; then
    echo_info "libk4a installed"
else
    echo_warn "libk4a not found"
fi

if [ -f "/usr/include/k4a/k4a.h" ]; then
    echo_info "k4a headers installed: /usr/include/k4a/"
else
    echo_warn "k4a headers not found"
fi

echo ""
echo_info "To test the installation:"
echo "  1. Connect your Azure Kinect"
echo "  2. Run: k4aviewer"
echo ""
echo_info "For WSL2 users:"
echo "  Make sure to attach USB devices first:"
echo "    usbipd attach --wsl --busid <BUSID>"
echo ""
echo_info "If you encounter permission issues, log out and back in"
echo "  (to apply plugdev group membership)"
echo ""
