# Nereo PoliTOcean
The code for the ROV Nereo made by PoliTOcean.
The src folder is the src folder of the ros2 workspace.
We are currently using ROS2 Humble.
Inside the src folder there are two packages:
1. nereo_pkg
    This package contains the code for all the nodes to be run on both the ROV and the control station, as well as the two launch files to run on the two devices.
2. nereo_interfaces
    This package contains all the custom messages and services provided by the ROV.
## Setup instructions
### I will do an installation script to automatically setup the workspace and the packages, but as of now you'll have to do it manually.
1. Create a ROS2 workspace: navigate to your desired folder and run:
    ```
    mkdir -p ros2_ws/src
    cd ros2_ws
    colcon build
    ```
2. Create the packages, in the root folder of the workspace run:
    ```
    cd src
    ros2 pkg create --build-type ament_python nereo_pkg
    ros2 pkg create nereo_interfaces
    ```
3. Add the code to the packages:
    > Note: when specifying the paths, the paths preceeded by an uppercase R will be relative paths referring to this Repository, the paths preceeded by an uppercase W will be relative paths referring to the workspace created in step 1.
    
    Now copy the folders R"nereo_pkg/launch" and R"nereo_pkg/nereo_pkg" inside your newly created python package (W"src/nereo_pkg"), as well as the R"nereo_pkg/package.xml" and R"nereo_pkg/setup.py" files. Replace the already existing files, if any.
    Now copy the folders R"nereo_interfaces/msg" and R"nereo_interfaces/srv" inside your newly created cmake package (W"src/nereo_interfaces"), as well as the R"nereo_interfaces/package.xml" and R"nereo_interfaces/CMakeLists.txt" files. Replace the already existing files, if any. Inside W"src/nereo_interfaces/", make sure to delete the folders "include" and "src".
4. Move to the root directory of your workspace and run:
    ```
    colcon build
    ```
    All the packages should now be built, make sure to include them in your .bashrc file:
    ```
    echo source install/setup.bash > ~/.bashrc ## this line is optional, it will make sure that the workspace is automatically loaded every time you open a new terminal.
    source install/setup.bash
    ```