# This Dockerfile is only used to build the base image for the project in order to avoid
# Unnecessarily long build times.
# It is a Debian-based Python3 image.
# It contains all the necessary python packages to run the CLAMOUR project.

FROM balenalib/raspberrypi3-debian-python:3.6.8-buster

# Some base python packages must be installed through 
# apt-get because installing them with pip fails.
RUN apt-get update

# Some base packages are needed just to install pygame
RUN apt-get install build-essential python3-dev python3-setuptools python3-opengl python3-numpy python3-scipy \
    python3-matplotlib python3-pygame

COPY requirements.txt /
RUN pip install --upgrade pip setuptools wheel
RUN pip install --no-cache-dir -r requirements.txt

# Append python3.6 packages to PYTHONPATH so that python3.7 can use them.
ENV PYTHONPATH="$PYTHONPATH:/usr/local/lib/python3.6/site-packages"
