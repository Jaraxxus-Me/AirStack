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

## Connect and Control Spot
1. Connect the Spot WIFI
    `spot-BD-xxxxxxxxxxx`
    ```shell
    ping 192.168.80.3
    ```
    Then, power on the robot with e-stop.
2. Test power

    WARNING: Here the Spot will stand up and take a picture, please don't be too close to it.
    Inside the docker container, run
    ```shell
    bash src/spot_ros2/starter.sh
    ```
    This will use Spot Python scripts to test the power of Spot.
    #### Quick Trouble Shooting
    - Hello_spot will fail because there is not an E-Stop endpoint.
        ```shell
        2021-03-30 15:26:36,283 - ERROR - Robot is E-Stopped. Please use an external E-Stop client, such as
        the E-Stop SDK example, to configure E-Stop.
        ```
    - If you see the following error:
        ```shell
        $ python3 hello_spot.py 192.168.80.3
        2021-04-03 15:10:28,189 - ERROR - Hello, Spot! threw an exception: InvalidLoginError()
        ```
      Your username or password in `src/spot_ros2/starter.sh` is incorrect. Check your spelling and verify your credentials with your robot administrator.
    - If you see the following error:
        ```shell
        2024-11-25 01:45:36,409 - ERROR - Hello, Spot! threw an exception: KeepaliveMotorsOffError()
        ```
      Try to use the remote control to turn on Spot power, then turn off and release control.
    - If you see the following error:
        ```shell
        2024-11-25 01:46:36,082 - ERROR - Hello, Spot! threw an exception: ResourceAlreadyClaimedError()
        ```
      Double check 1. The ROS2 Spot Driver is not connected to this spot and running. 2. The remote control has released control.
3. Use ROS2 services and nodes

    In one tmux session of the docker container, turn on Spot Driver
    ```shell
    ros2 launch spot_driver spot_driver.launch.py config_file:=/root/ros_ws/src/spot_ros2/spot_driver/config/spot_ros_airlab.yaml
    ```
    Check that you see this message:
    ```shell
    [spot_ros2-1] [INFO] [1732499976.322883784] [spot_ros2]: Driver successfully started!
    ```
    Keep this running and in another tmux session, verify that you can see the spot services and topics running:
    ```shell
    ros2 service list
    ros2 topic list
    ```
    Let's test some of them:
    - Make Spot Stand
        ```shell
        [robot_1]root@e4e356aeaabb:~/ros_ws# ros2 service call /stand std_srvs/srv/Trigger
        waiting for service to become available...
        requester: making request: std_srvs.srv.Trigger_Request()

        response:
        std_srvs.srv.Trigger_Response(success=True, message='Success')
        ```
    - Make Spot Sit
        ```shell
        [robot_1]root@e4e356aeaabb:~/ros_ws# ros2 service call /sit std_srvs/srv/Trigger
        waiting for service to become available...
        requester: making request: std_srvs.srv.Trigger_Request()

        response:
        std_srvs.srv.Trigger_Response(success=True, message='Success')
        ```
    - Make Spot walk forward 1m and stop
        ```shell
        [robot_1]root@e4e356aeaabb:~/ros_ws# ros2 run spot_examples walk_forward
        [INFO] [1732500254.566473227] [walk_forward]: (logging.spot_examples.walk_forward.WalkForward) Robot name: None
        [INFO] [1732500254.566495369] [walk_forward]: (logging.spot_examples.walk_forward.WalkForward) Claiming robot
        [INFO] [1732500254.736722470] [walk_forward]: (logging.spot_examples.walk_forward.WalkForward) Claimed robot
        [INFO] [1732500254.736787413] [walk_forward]: (logging.spot_examples.walk_forward.WalkForward) Powering robot on
        [INFO] [1732500254.959164901] [walk_forward]: (logging.spot_examples.walk_forward.WalkForward) Standing robot up
        [INFO] [1732500255.325351108] [walk_forward]: (logging.spot_examples.walk_forward.WalkForward) Successfully stood up.
        [INFO] [1732500255.325394460] [walk_forward]: (logging.spot_examples.walk_forward.WalkForward) Walking forward
        [INFO] [1732500255.327267909] [walk_forward]: Sending Action [walk_forward]
        [INFO] [1732500255.340064060] [walk_forward]: Action accepted
        [INFO] [1732500258.154378392] [walk_forward]: Finished successfully
        [INFO] [1732500258.154764432] [walk_forward]: (logging.spot_examples.walk_forward.WalkForward) Successfully walked forward
        ```
    - Make the Spot Arm Stretch
        ```
      WARNING: Please be away from Spot, as its arm will be stretched
      [robot_1]root@e4e356aeaabb:~/ros_ws# ros2 run spot_examples arm_simple
        ```
