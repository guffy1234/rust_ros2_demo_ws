#!/bin/bash

# ROS 2 Jazzy Installation Script for Ubuntu 24.04 (Noble)
# Based on official docs: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
# Run this script with sudo privileges if needed, but most commands use sudo internally.
# Assumes you are on Ubuntu 24.04. Check with: lsb_release -a

set -e  # Exit on any error

echo "Starting ROS 2 Jazzy installation..."

# 1. Ensure UTF-8 Locale Support
echo "Configuring UTF-8 locale..."
locale_check=$(locale | grep -i utf8 || echo "No UTF-8")
if [[ ! $locale_check ]]; then
    echo "UTF-8 not detected. Configuring..."
    sudo apt update && sudo apt install locales -y
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
else
    echo "UTF-8 locale already configured."
fi
locale  # Verify

# 2. Enable Ubuntu Universe Repository
echo "Enabling Ubuntu Universe repository..."
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

# 3. Install curl if needed (for ros2-apt-source)
echo "Installing curl..."
sudo apt update && sudo apt install curl -y

# 4. Install ros2-apt-source to Configure ROS 2 Repositories
echo "Setting up ROS 2 apt source..."
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
rm /tmp/ros2-apt-source.deb

# 5. (Optional) Install Development Tools - Uncomment if needed
# echo "Installing development tools..."
# sudo apt update && sudo apt install ros-dev-tools -y

# 6. Update Package Index and System
echo "Updating package index and upgrading system..."
sudo apt update
sudo apt upgrade -y



# 7. Install ROS 2 Jazzy Desktop (Recommended) - Change to ros-jazzy-ros-base for minimal install
echo "Installing ROS 2 Dev Tools..."
sudo apt install ros-dev-tools -y
echo "Installing ROS 2 Jazzy Desktop..."
sudo apt install ros-jazzy-desktop -y
# Install build essentials and clang/llvm
sudo apt install build-essential clang libclang-dev llvm-dev -y
# Also install ROS2 development tools if not already installed
sudo apt install ros-jazzy-rcl ros-jazzy-rcutils -y

# 8. Set Up the ROS 2 Environment Automatically
echo "Setting up ROS 2 environment in ~/.bashrc..."
echo "# Enable ROS 2 Jazzy" >> ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Source it for the current session
source /opt/ros/jazzy/setup.bash

echo "Installation complete!"
echo "To verify, open a new terminal and run:"
echo "  ros2 --help"
echo "For demo:"
echo "  # Terminal 1:"
echo "  ros2 run demo_nodes_cpp talker"
echo "  # Terminal 2:"
echo "  ros2 run demo_nodes_py listener"

# Optional: Verify in this script (will hang, so commented out)
# echo "Verifying installation... (Run demos manually)"