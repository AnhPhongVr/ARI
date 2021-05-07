# Steps

## Create a package

## Build package

* First of all, source your setup.<br>
  Be careful to be in the ari workspace<br>
  ```source ./devel/setup.bash```

* Then, use catkin to update ros executable files<br>
  Be careful to be in the ari workspace <br>
  ```catkin build```<br>
  If you know what you have to build because everything else is already built, then use this command<br>
  ```catkin build package_name```

## Run on Gazebo

* Launch Gazebo with the command below <br>
  ```roslaunch ari_gazebo ari_gazebo.launch public_sim:=true```
  
* chmod +x

* Launch the executable you need to <br>
  You need to know the executable name and the package where the executable is <br>
  Be aware that you have to type the extension '.py' for python files and not for c++ files <br>
  ```roslaunch package_name script_name```

## Run on ARI

* Use a FTP (File transfer Protocol) to deploy packages <br>
    In our case we use FileZilla because it's free and user friendly <br>
    You have to copy the "ari_isep" folder and replace it with the current one <br>

* Connect your pc to ARI with SSH <br>
    To ssh on ARI use this command <br>
    ```bash
    $ ssh 10.68.0.1 -y
    ```
    Place your working directory in the "ari_public_ws" folder <br>

* Build your code <br>
    To build all the packages use this command <br>
    ```bash
    $ catkin_make
    ```
    To build only one package use this command <br>
    ```bash
    $ catkin_make --only-pkg-with-deps [package]
    ```

* Source setup.bash file <br>
    To read and execute setup.bash use this command <br>
    ```bash
    $ source ./devel/setup.bash
    ```

* Kill conflicting node<br>
    On the ARI robot the node /play_motion is continuously running, it moves the arms <br>
    This node  may conflict with the nodes we are going to launch, it is better to kill it <br>
    ```bash
    $ rosnode kill /play_motion
    ```

* Run node on ARI <br>
    * To run a node with python script on ARI use this command <br>
    ```bash
    $ roscd [package_name]/scripts
    $ chmod +x [script_name].py 
    $ rosrun [package_name] [node_name].py
    ```
    * To run a node with c++ code on ARI use this command <br>
    ```bash
    $ rosrun [package_name] [node_name]
    ```
