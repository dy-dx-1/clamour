FROM balenalib/raspberry-pi-debian-python:3.7.2

# Some base python packages must be install through apt-get because
# installing them with pip fails.
RUN apt-get update && apt-get install python3-numpy python3-scipy python3-matplotlib

COPY requirements.txt /
RUN pip install -r requirements.txt

COPY /src /src
WORKDIR /src

CMD [ "python", "./clamour.py" ]
