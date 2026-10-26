# CLAMOUR

#### The name of this project, CLAMOUR, stands for Collaborative Localization Adapted from Modular Onboard UWB Ranger.

The objective of CLAMOUR is to create self-localizing audioguides for the Chambord Castle Museum's exposition (https://www.chambord.org/en/discovering/the-castle-visit/). These audioguides will generate music based on the visitor's position within the castle.

A prototype of this audioguide was realized in 2018. More details on this prototype and the research behind it can be found in the following paper: https://arxiv.org/abs/1905.03247

This project is realized in collaboration by the following laboratories: MIST - Polytechnique, NXI Gestatio - UQAM, INIT Robots - ETS.

#### Docker usage
A Dockerfile is available to use CLAMOUR inside a container.
The reasoning is that this way CLAMOUR can be deployed and updated easily on a cluster of Raspberry PI whilst guaranteeing us the configuration stays static and we never have discrepancies in configuration between nodes.

Since CLAMOUR uses a USB device (Pozyx), you need to give Docker access to the device when running the container.

Chances are the USB device in question will be `/dev/ttyACM0`, but you should make sure to find the correct one for your node.

Use the `--device` parameter with `docker run` command, like follows: `docker run --device=/dev/ttyACM0 clamour` where Pozyx device is `/dev/ttyACM0` and docker image name is `clamour`.

### Project Description
1. Objective 
The goal of this project is to achieve collaborative localization and tracking with minimal infrastructure

2. Key concept
UWB - ultra wide band
UWB is a radio technology used here to measure distances between devices. These mesurements are then used to estimate positions. 

Anchor
An anchor is a fixed reference point whose position is know. A device can measure its distance to an anchor to obtain information about its localisation. 

Tag/mobile device
a tag is a UWB device that the the system aims to localize. it can also communicate with neighboring tags and contribute to their localization.

TDMA - Time-Division Multiple Access
Time is divided into slots. A device can transmit during its assigned slots instead of transmitting at the same time as its neighbors.

3. the system's communication cycle
Step 1 - Initialization
The device first start to wait a random delay, then attempt to detect other tags around itself. It then update itself and switch to the next state
Step 2 - Synchronisation
This state prepares the tag for synchronization by converging the logical clock of the tags so that they can coordonnate their transmissions. Afte the preparations are done, the device switches to the Scheduling state

Step 3 - Scheduling
This state attributes slots for each tag using the temporal informations prepared during the synchronization state. When the planification windows is finished, it switches to the Listen state. If an error message is returned, it switches to the Synchronization state.

Step 4 - Listen
This state can be divided in four different outcomes. If a re-synchronisation is needed, the device switches to the Synchronisation state. If the TDMA cycle is over, it also switches to the Synchronization state. If were in the cycle and the current slot is owned by the device, it switch to the Task state. Finally, if were in the cycle but the current slot is not owned by the device, it stays in the Listen state.

Step 5 - Task
This is the step start by discovering periodically the other devices. It also cleans the obsolete informations by the mean of a garbage collector. After the other tags have been discovered, the program starts mesuring distances between devices and transmit the result. If the surrondings have changed, the device sends a message to the other devices about the change and then updates the logical clock and the delay with the surrounding device. The distance estimatior receives : 
- the distance mesured between the device and the anchors
- the distance mesured between the device and the other tags
- the logical clock and the delay
- the known topologic of the network


