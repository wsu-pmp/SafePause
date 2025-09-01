#!/bin/bash

set -e

# Set ROS distribution if not set
export ROS_DISTRO=${ROS_DISTRO:-humble}


# deb-conf pre-seed config
sudo debconf-set-selections <<EOF
tzdata tzdata/Areas select Australia
tzdata tzdata/Zones/Australia select Sydney
EOF


# utf-8 locale
if [[ "$LANG" != "en_US.UTF-8" ]] || [[ "$LC_ALL" != "en_US.UTF-8" ]]; then
    echo ">>> Setting up en_US.UTF-8 locale"
    sudo apt-get update
    sudo apt-get install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
else
    echo ">>> en_US.UTF-8 locale already configured"
fi


# Basic dependencies
echo ">>> Installing basic dependencies"
sudo apt-get update
sudo apt-get install -y software-properties-common ca-certificates curl wget


# Universe repository
if ! grep -q "^deb.*universe" /etc/apt/sources.list /etc/apt/sources.list.d/*.list 2>/dev/null; then
    echo ">>> Adding universe repository"
    sudo add-apt-repository universe -y
else
    echo ">>> Universe repository already enabled"
fi


# ROS 2 GPG key
if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
    echo ">>> Adding ROS 2 GPG key"
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
else
    echo ">>> ROS 2 GPG key already exists"
fi


# ROS 2 repository to sources list
if ! grep -q "packages.ros.org/ros2" /etc/apt/sources.list.d/ros2.list 2>/dev/null; then
    echo ">>> Adding ROS 2 repository to sources list"
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt-get update
    sudo apt-get upgrade -y
else
    echo ">>> ROS 2 repository already exists in sources list"
fi


# ROS 2 desktop
echo ">>> Installing ROS 2 desktop"
sudo apt-get clean
sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-$ROS_DISTRO-desktop \
    ros-dev-tools


# ROS 2 general dependencies
echo ">>> Installing ROS 2 general dependencies"
sudo apt-get install -y \
    libpoco-dev \
    libyaml-cpp-dev \
    dbus-x11


# ROS 2 specific packages
echo ">>> Installing $ROS_DISTRO packages"
sudo apt-get install -y \
    ros-$ROS_DISTRO-control-msgs \
    ros-$ROS_DISTRO-realtime-tools \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-gazebo-msgs \
    ros-$ROS_DISTRO-moveit-msgs \
    ros-$ROS_DISTRO-ign-ros2-control \
    ros-$ROS_DISTRO-moveit-configs-utils \
    ros-$ROS_DISTRO-moveit-ros-move-group


# Install Docker if not installed
if ! command -v docker &> /dev/null; then
    echo ">>> Installing Docker"
    # Docker GPG key
    sudo install -m 0755 -d /etc/apt/keyrings
    sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
    sudo chmod a+r /etc/apt/keyrings/docker.asc

    # Add Docker repository
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

    # Install Docker packages
    sudo apt-get update
    sudo apt-get install -y \
        docker-ce \
        docker-ce-cli \
        containerd.io \
        docker-buildx-plugin \
        docker-compose-plugin
else
    echo ">>> Docker already installed"
fi


# Install moveit
echo ">>> Installing moveit"
sudo apt-get install -y ros-$ROS_DISTRO-moveit


# Install rosdep
echo ">>> Installing rosdep"
sudo apt-get install -y python3-rosdep


# Initialize rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo ">>> Initializing rosdep"
    sudo rosdep init
else
    echo ">>> rosdep already initialized"
fi


# Update rosdep and install dependencies
echo ">>> Updating rosdep and installing dependencies"
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep update
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y


# Add ROS 2 environment setup to bashrc
if ! grep -q "source /opt/ros/\$ROS_DISTRO/setup.bash" ~/.bashrc; then
    echo ">>> Adding ROS 2 environment sourcing to bashrc"
    echo "" >> ~/.bashrc
    cat <<EOL >> ~/.bashrc
# source ros2
source /opt/ros/\$ROS_DISTRO/setup.bash

# set locale in session (potentially redundant)
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8
EOL
else
    echo ">>> ROS 2 already sourced in bashrc"
fi


# Clean up
sudo apt-get autoremove -y
sudo apt-get autoclean

echo ""
echo "Setup complete!"
echo "ROS_DISTRO is set to $ROS_DISTRO."
echo ""
echo "    Run 'source ~/.bashrc' or restart your terminal to use ROS 2."
echo "    Verify installation with 'ros2 --help'."
echo ""
