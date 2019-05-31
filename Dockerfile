FROM arm32v6/python:3-alpine3.9

COPY requirements.txt /
RUN pip install -r requirements.txt

COPY /src /src
WORKDIR /src

CMD [ "python", "./clamour.py" ]
