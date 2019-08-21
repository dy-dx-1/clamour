FROM samsei/arm:latest

# the mdkir is required for the log volume mounted by docker-compose
RUN mkdir -p /var/log/clamour
COPY /src /src
WORKDIR /src

# It is necessary to use python 3.7 specifically
CMD python3.7 ./clamour.py > /var/log/clamour/logs.txt
