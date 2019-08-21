FROM samsei/arm:latest

COPY /src /src
WORKDIR /src

# It is necessary to use python 3.7 specifically
CMD python3.7 ./clamour.py > /var/log/clamour/logs.txt
