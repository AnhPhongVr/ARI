# Lexique

* **Packages :** Packages are the software organization unit of ROS code. Each package can contain libraries, executables, scripts, or other artifacts.

* **Manifests (package.xml) :** A manifest is a description of a package. It serves to define dependencies between packages and to capture meta information about the package like version, maintainer, license, etc... 

* **Stacks :** A stack is a collection of packages that form a library. 

* **Stack Manifest :** A manifest but for a stack.

* **Nodes :** A node is an executable that uses ROS to communicate with other nodes.

* **Messages :** A ROS data type, used when subscribing or publishing to a topic.

* **Topics :** Nodes can post messages to a topic but also subscribe to a topic in order to receive messages.from it.

* **Master :** Name service for ROS (i.e. helps nodes find each other).

* **rosout :** The equivalent under ROS of stdout/stderr.

* **roscore :** A roscore is a collection of nodes and programs that are pre-requisites of a ROS-based system. You must have a roscore running in order for ROS nodes to communicate. It is composed of Master + rosout + parameter server 

* **msg :** A msg files are simple text files that describe the fields of a ROS message. They are used to generate source code for messages in different languages.

* **srv :** An srv file describes a service. It is composed of two parts: a request and a response.

* **service :** Services are another means of communication between nodes. Services allow nodes to send a request and receive a response. rosservice allows to easily implement services connected to the clients or services of the ROS framework.
