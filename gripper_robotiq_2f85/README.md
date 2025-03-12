# Server for the Robotiq 2f85 Gripper

Works with the UR both in remote and local mode.

## Assembling 

Follow the instructions in the box.

## URCaps

For some reasons, the Robotiq URCaps is not working correctly with any form of remote control. The 
only possibility is to install the RS485 URCaps (currently version 1.0.0) and connect to the gripper
through a socket as in the server. 

## Prerequisites

To correctly execute this script, one has to install [socat](http://www.dest-unreach.org/socat/):

- Ubuntu and Debian-like systems:
    ```shell
    sudo apt-get install -y socat
    ```

- MacOS:
    ```shell
    brew install socat
    ```

Also install the requirements in `requirements.txt`. 

## ATTENTION

In some occasions, if you disconnect from the UR with the gripper closed, then initialize the 
connection again and open the gripper, it may happen that it: opens, closes and opens. Be warned of
this.