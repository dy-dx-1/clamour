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