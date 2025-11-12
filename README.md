# pyADSThesis


## Get started

This project requires [ROS 2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation.html) and Ubuntu 24.04 to run.

First source ROS2, usually:
```bash
source /opt/ros/jazzy/setup.bash
```
Inside the prometheus\_req\_ws run
```bash
./build_and_launch.sh
```
It will fail, ignore the error.
Now run
```bash
source ./install/source.bash
./fix_ext_lib.bash
./build_and_launch.bash
```

You are ready to go.


## How to use

Set up with the correct ips src/prometheus_req_bringup/config/ads_node_config.yaml.

Then, inside prometheus\_req\_ws run:
```bash
./launch.bash
```

In case it doesn't exist, the program will create a route
