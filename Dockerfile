FROM balenalib/raspberrypi3

WORKDIR /usr/src/app

COPY ./ ./
RUN pip3 install -r ./requirements.txt

CMD python3 ./src/clamour.py