# JPO package 

In this file you will find all the instructions to run easly all the code <br> 

## scripts folder

### control/ari_with_controller.py

On Ari you will do this instructions to run the program : <br>

* Run this command to see the joy topic and verify that the controller is on <br>
    ```bash
    $ rostopic echo joy
    ```

* Run this command to launch the code <br>
    ```bash
    $ rosrun jpo ari_with_contoller.py
    ```

### welcome/run_welcome_person.py

On Ari you will do this instructions to run the program : <br>

* In one terminal run this command <br>
    ```bash
    $ roslaunch pal_person_detector_opencv detector.launch image:=/head_front_camera/image_raw
    ```

* In another terminal run this command <br>
    ```bash
    $ rosrun jpo run_welcome_person.py
    ```

## src folder
