# ROS melodic Cheat Sheet 

## Worspace

* Creates a catkin workspace named "catkin_ws"
    ```bash
    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/src
    $ catkin_init_workspace
    ```

* Builds the workspace
    ```bash
    $ cd ~/catkin_ws/
    $ catkin_make
    $ source devel/setup.bash
    ```

## ROS Filesystem

* Gets information about the packages
    ```bash
    $ rospack find [package_name]
    $ rosstack find [stack_name]
    ```

* Goes directly inside the folder of a package or a stack
    ```bash
    $ roscd [locationname[/subdir]]
    ```

* Goes directly inside the folder that contains log
    ```bash
    $ roscd log
    ```

* Serves to execute the ls command inside a package
    ```bash
    $ rosls [package_name[/subdir]]
    ```

## Package

* Creates a package
    ```bash
    $ cd ~/catkin_ws/src
    $ catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
    ```

* Builds a package
    ```bash
    $ catkin_make [make_targets] [-DCMAKE_VARIABLES=...]
    ```

## Running system
 
* Runs roscore
    ```bash
    $ roscore
    ```

* Lists information about running ROS nodes
    ```bash
    $ rosnode list
    ```

* Returns information concerning the node indicated in parameter
    ```bash
    $ rosnode info /rosout
    ```

* Launches a node present in a package
    ```bash
    $ rosrun [package_name] [node_name]
    ```

* Verifies that the node is really active
    ```bash
    $ rosnode ping [node_name]
    ```

* Starts nodes as defined in a launch file
    ```bash
    $ roslaunch [package] [filename.launch]
    ```

## Topics

* Shows the currently active nodes and topics via a dynamic graph
    ```bash
    $ rosrun rqt_graph rqt_graph
    ```

* Shows the data published by a Topic
    ```bash
    $ rostopic echo [topic]
    ```

* Shows a list of publishing and subscribing topics and their types
    ```bash
    $ rostopic list -v
    ```

* Returns the message type of each published topic
    ```bash
    $ rostopic type [topic]
    ```

* Views message details
    ```bash
    rosmsg show [topic]
    ```

* Publishes data to an active topic
    ```bash
    $ rostopic pub [topic] [msg_type] [args]
    ```

* Reports the rate at which data is published
    ```bash
    $ rostopic hz [topic]
    ```

## Services

* Shows the services provided
    ```bash
    $ rosservice list
    ```

* Examines a service in more detail
    ```bash
    $ rosservice type [service]
    ```

* Calls a service
    ```bash
    $ rosservice call [service] [args]
    ```

## Debug 

* Launches rqt_console (display output from nodes)
    ```bash
    $ rosrun rqt_console rqt_console
    ```

* Launches rqt_logger_level (change the verbosity level of nodes)
    ```bash
    $ rosrun rqt_logger_level rqt_logger_level
    ```

* Examines your system to try and find problems (not a joke ^^)
    ```bash
    $ roswtf
    ```
    
## Parameters
