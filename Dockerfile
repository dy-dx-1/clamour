FROM samsei/arm:latest

COPY /src /src
WORKDIR /src

CMD ["python", "./clamour.py"]
