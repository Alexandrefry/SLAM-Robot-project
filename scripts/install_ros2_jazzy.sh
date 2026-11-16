#!/usr/bin/env bash
# ==============================================================================
# Script d'installation automatisé de ROS 2 Jazzy (Ubuntu 24.04 LTS / aarch64)
# Projet SLAM Robotique avec LiDAR LD06
# ==============================================================================

set -e

echo "=========================================================="
echo " [1/6] Vérification du système..."
echo "=========================================================="
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$UBUNTU_CODENAME" != "noble" ]; then
        echo "Attention : Ce script est conçu pour Ubuntu 24.04 (noble). Système détecté : $UBUNTU_CODENAME"
    fi
fi

# Configuration des locales UTF-8
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "=========================================================="
echo " [2/6] Ajout des dépôts et clés GPG officiels ROS 2..."
echo "=========================================================="
sudo apt install -y software-properties-common curl gnupg
sudo add-apt-repository -y universe

# Clé d'authentification ROS 2
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Dépôt ROS 2 Jazzy
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "=========================================================="
echo " [3/6] Installation de ROS 2 Jazzy Desktop & SLAM Toolbox..."
echo "=========================================================="
sudo apt update
sudo apt install -y \
    ros-jazzy-desktop \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-teleop-twist-keyboard \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-serial

echo "=========================================================="
echo " [4/6] Initialisation de rosdep..."
echo "=========================================================="
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update || true

echo "=========================================================="
echo " [5/6] Configuration des droits série pour le LiDAR..."
echo "=========================================================="
# Ajout de l'utilisateur au groupe dialout pour accéder à /dev/ttyUSB0 sans sudo
sudo usermod -aG dialout "$USER"

echo "=========================================================="
echo " [6/6] Configuration de l'environnement bash..."
echo "=========================================================="
ROS_SETUP="source /opt/ros/jazzy/setup.bash"
if ! grep -q "$ROS_SETUP" ~/.bashrc; then
    echo "$ROS_SETUP" >> ~/.bashrc
    echo "-> ROS 2 Jazzy ajouté à ~/.bashrc"
fi

echo ""
echo "=========================================================="
echo "✅ Installation de ROS 2 Jazzy terminée avec succès !"
echo "=========================================================="
echo "Pour activer ROS 2 dans ce terminal : source /opt/ros/jazzy/setup.bash"
