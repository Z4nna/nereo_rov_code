#!/bin/bash
echo "Welcome to the installer of Nereo's ROS package!"
read -p "Would you like to proceed?"
read -p "Enter the name of the cloned repository: " repo_name
source /opt/ros/humble/setup.bash
cd ..
mkdir -p ros2_nereo_ws/src
cd ros2_nereo_ws
colcon build
# creates packages
cd src
ros2 pkg create --build-type ament_python nereo_pkg
ros2 pkg create nereo_interfaces
mkdir -p nereo_pkg/launch
# adds nereo_pkg code
cp -afv ../../$repo_name/src/nereo_pkg/launch/. ./nereo_pkg/launch/
cp -afv ../../$repo_name/src/nereo_pkg/nereo_pkg/. ./nereo_pkg/nereo_pkg/
cp -afv ../../$repo_name/src/nereo_pkg/package.xml ./nereo_pkg/
cp -afv ../../$repo_name/src/nereo_pkg/setup.py ./nereo_pkg/
# adds nereo_interfaces code
cp -afv ../../$repo_name/src/nereo_interfaces/msg/. ./nereo_interfaces/msg/
cp -afv ../../$repo_name/src/nereo_interfaces/srv/. ./nereo_interfaces/srv/
cp -afv ../../$repo_name/src/nereo_interfaces/package.xml ./nereo_interfaces/
cp -afv ../../$repo_name/src/nereo_interfaces/CMakeLists.txt ./nereo_interfaces/
rm -rfv ./nereo_interfaces/include
rm -rfv ./nereo_interfaces/src
# builds and sources the packages
cd ..
colcon build
source ./install/setup.bash
read -p "Would you like to permanently source the setup script? You won't have to source it again for every terminal you open. (YES / no): " source_setup
if [ $source_setup == ""];
then
	$source_setup = "YES"
fi
if [ $source_setup == "YES" ];
then
	echo source ./install/setup.bash >> ~/.bashrc
fi
echo "Nereo packages are successfully installed!"
echo "You can now delete the previously cloned repository."
read -p "Press any key to exit."

