# Steps

##

## ARI side

* Use a FTP (File transfer Protocol) to deploy packages <br>
    In our case we use FileZilla because it's free and user friendly <br>
    You have to copy the "ari_isep" folder and replace it with the current one <br>

* Connect your pc to ARI with SSH <br>
    To ssh on ARI use this command <br>
    ```bash
    $ ssh 10.68.0.1
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
    On the ARI robot the node ??? is continuously running, it moves the arms <br>
    This node  may conflict with the nodes we are going to launch, it is better to kill it <br>
    ```bash
    $ rosnode kill ???
    ```

* Run node on ARI <br>
    To run a node on ARI use this command <br>
    ```bash
    $ rosrun [package_name] [node_name]
    ```
