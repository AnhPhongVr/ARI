# Reference document on ARI

## ROS

### Introduction

**ROS** (Robot Operation System) is an operating system used for robots which is widely used around the globe. It only runs on Unix-based plateforms and it is the OS used for **ARI**.

### Distribution

The distribution used for **ARI** is the *Melodic* one.

### ROS directory

  The **ROS** directory can be found at *~/ari/src*, it is linked by **Catkin** and you will found every usefull files you need. It contains a **ROS [packages](http://wiki.ros.org/Packages)** which will allow you to easily work on **ARI** by having plenty of already functionnals tutorials. **This is where you will code**. You will find in the **Catkin** part of this document how to create your own package.

### Node / Topic concept

  It is the storage/communication part of the robot. This principle is **extremely important**. ROS works around topics that store the robot state. Each topic is assigned to a specific part of the robot (head, arm, ...) and the nodes have to write / read this informations in order to make the robot works via *publisher*/*subscriber* system.
      
  [Nodes](http://wiki.ros.org/Nodes) : Thanks to **Catkin**, you can launch a **package** which will create a **node** from the source code contained in it.
  
  [Topics](http://wiki.ros.org/Topics) : This is where **nodes** find informations about the state of **ARI** (camera, motors, etc.). A **node** has to *subscribe* to every topics it wants to read and to *publish* ti every topics it wantes to write. The command [rostopic](http://wiki.ros.org/rostopic) will help you to find topics you will need for your code.
  
  A node can *subscribe* to several topics. Several nodes can *publish* the same topic.

### Hardware

| Hardware     | Topics associated | Information |
|--------------|:-----------------:|:------------|
| Head Camera  |                   |             |
| Torso camera |                   |             |
| Rear camera  |                   |             |
| Speakers     |                   |             |
| Microphones  |                   |             |
| Arms         |                   |             |
| Hands        |                   |             |
| Head         |                   |             |
| Wheels       |                   |             |

### Catkin

### Gazebo

**Gazebo** is a 3D simulator which is able to simulate your code and moreover, it will allow to create your own virtual room with several 3D objects like balls or chairs.

## General information on ARI

## Other technical information

Add here links to the cheat sheet and any other `.md` file that will be created in this repository.

## Versionning

| Version |              Modification             |       Date |
|---------|:-------------------------------------:|-----------:|
| 0.1     |        Table of contents added        | 04/03/2021 |
| 0.2     | ROS documentation partially completed | 05/03/2021 |
| 0.3     |                                       |            |
