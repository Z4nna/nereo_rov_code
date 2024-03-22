#!/bin/bash

# Function to check if the command executed successfully
check_success() {
    if [ $? -ne 0 ]; then
        echo "Error: $1 failed. Exiting."
        exit 1
    fi
}

# Function to create ROS packages
create_ros_packages() {
    ros2 pkg create --build-type ament_python nereo_pkg
    ros2 pkg create nereo_interfaces
}

# Function to copy code and files from cloned repository
copy_code_from_repo() {
    cp -afv ../$repo_name/src/nereo_pkg/launch/. ./src/nereo_pkg/launch/
    cp -afv ../$repo_name/src/nereo_pkg/nereo_pkg/. ./src/nereo_pkg/nereo_pkg/
    cp -afv ../$repo_name/src/nereo_pkg/package.xml ./src/nereo_pkg/
    cp -afv ../$repo_name/src/nereo_pkg/setup.py ./src/nereo_pkg/

    cp -afv ../$repo_name/src/nereo_interfaces/msg/. ./src/nereo_interfaces/msg/
    cp -afv ../$repo_name/src/nereo_interfaces/srv/. ./src/nereo_interfaces/srv/
    cp -afv ../$repo_name/src/nereo_interfaces/package.xml ./src/nereo_interfaces/
    cp -afv ../$repo_name/src/nereo_interfaces/CMakeLists.txt ./src/nereo_interfaces/
    rm -rfv ./src/nereo_interfaces/include
    rm -rfv ./src/nereo_interfaces/src
}

# Function to build ROS packages
build_ros_packages() {
    colcon build
}

# Function to source ROS setup script
source_ros_setup() {
    source ./install/setup.bash
}

# Function to add setup script to .bashrc
add_to_bashrc() {
    echo "source ./install/setup.bash" >> ~/.bashrc
}

# Function to display installation completion message
display_completion_message() {
    echo "Nereo packages are successfully installed!"
    echo "If encountering any error during usage, check the permissions :)"
    echo "You can now delete the previously cloned repository."
    read -p "Press any key to exit."
}

echo "Welcome to the installer of Nereo's ROS package!"

# Validate user input for proceeding
read -p "Would you like to proceed? (yes/no): " proceed
if [[ ! $proceed =~ ^[Yy][Ee][Ss]|[Yy]$ ]]; then
    echo "Installation aborted."
    exit 0
fi

# Validate user input for repository name
read -p "Enter the name of the cloned repository: " repo_name
if [ ! -d "../$repo_name" ]; then
    echo "Error: Repository not found. Make sure you have entered the correct name."
    exit 1
fi

# Source ROS setup script
source /opt/ros/humble/setup.bash
check_success "Sourcing ROS setup script"

# Create ROS workspace and packages
mkdir -p ../ros2_nereo_ws/src
cd ../ros2_nereo_ws
colcon build
cd src
create_ros_packages
check_success "Creating ROS packages"

# Copy code from cloned repository
cd ..
copy_code_from_repo
check_success "Copying code from repository"

# Build ROS packages
build_ros_packages
check_success "Building ROS packages"

# Source ROS setup script
source_ros_setup

# Ask user to add setup script to .bashrc
read -p "Would you like to permanently source the setup script? You won't have to source it again for every terminal you open. (yes/no): " source_setup
if [[ $source_setup =~ ^[Yy][Ee][Ss]|[Yy]$ ]]; then
    add_to_bashrc
    echo "Setup script added to .bashrc."
fi

# Display completion message
display_completion_message
