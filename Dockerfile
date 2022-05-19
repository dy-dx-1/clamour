FROM arm64v8/ubuntu

WORKDIR /usr/src/app

RUN apt update && \
	apt install python3-pip -y

COPY ./ ./
RUN pip3 install -r ./requirements.txt

CMD python3 ./src/clamour.py
