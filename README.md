# Nereo PoliTOcean
The code for the ROV Nereo made by PoliTOcean.
The src folder is the src folder of the ros2 workspace.
We are currently using ROS2 Humble.
Inside the src folder there are two packages:
1. nereo_pkg
    This package contains the code for the python nodes to be run on both the ROV and the control station, as well as the two launch files to run on the two devices.
2. nereo_sensors_pkg
   This package contains the code for the C++ nodes, used to communicate with the I2C sensors.
## Setup instructions
### Option 1: manual installation
1. In the cloned folder, run:
   ```bash
    colcon build
    ```
    All the packages should now be built, you may want to include them in your .bashrc file:
    ```bash
    source install/setup.bash
    ```
