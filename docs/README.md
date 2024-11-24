# AirStack-Spot

This repo aims at integrating Spot2 ROS2 and AirStack.

## Installation

1. AirStack Docker and Code
    ```shell
    git clone https://github.com/Jaraxxus-Me/AirStack.git
    ```

    ```shell
    cd AirStack/
    docker login airlab-storage.andrew.cmu.edu:5001
    ## <Enter your andrew id (without @andrew.cmu.edu)>
    ## <Enter your andrew password>

    ## Pull the images in the docker compose file
    docker compose pull
    ```

    ```shell
    xhost +  # allow docker access to X-Server

    # Make sure you are in the AirStack directory.

    # Start docker compose services. This launches Isaac Sim and the robots.
    #  You can append `--scale robot=[NUM_ROBOTS]` for more robots, default is 1
    docker compose up -d
    ```

2. Spot Code
    ```shell
    cd [AirStack]/robot/ros_ws/src
    git clone https://github.com/Jaraxxus-Me/spot_ros2.git
    ```

3. Use Spot ROS2 in Docker
    ```shell
    docker exec -it airstack-robot-1 bash
    ```
    Inside docker, build ROS2 packages.
    ```shell
    [robot_1]root@e4e356aeaabb:~/ros_ws# source /opt/ros/humble/setup.bash
    [robot_1]root@e4e356aeaabb:~/ros_ws# colcon build --symlink-install --packages-ignore proto2ros_tests
    [robot_1]root@e4e356aeaabb:~/ros_ws# source install/local_setup.bash
    ```
    Lauch Spot Driver (it is something like ROS core)
    ```shell
    [robot_1]root@e4e356aeaabb:~/ros_ws# ros2 launch spot_driver spot_driver.launch.py config_file:=/root/ros_ws/src/spot_ros2/spot_driver/config/spot_ros_airlab.yaml
    ```
    You should expect:
    ```shell
    [INFO] [launch]: All log files can be found below /root/.ros/log/2024-11-24-21-50-53-990481-e4e356aeaabb-35119
    [INFO] [launch]: Default logging verbosity is set to INFO
    Failed to communicate with robot: ProxyConnectionError: The proxy on the robot could not be reached.
    Ensure the robot is powered on and you can ping 192.168.80.3. Robot may still be booting. Will retry in 15 seconds
    Failed to communicate with robot: ProxyConnectionError: The proxy on the robot could not be reached.
    Ensure the robot is powered on and you can ping 192.168.80.3. Robot may still be booting. Will retry in 15 seconds
    Failed to communicate with robot: ProxyConnectionError: The proxy on the robot could not be reached.
    ```
