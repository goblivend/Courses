FROM ubuntu:latest AS builder
WORKDIR /app
RUN apt update
RUN apt-get install -y cmake build-essential
RUN apt install -y \
              libboost-all-dev \
              qtbase5-dev qt5-qmake
COPY ./CMakeLists.txt .
COPY include ./include
COPY src ./src
RUN mkdir build && cd build && cmake .. && make -j

FROM ubuntu:latest
WORKDIR /app
COPY --from=builder /app/build/tele .
COPY images ./images
RUN apt-get update && apt-get install -y \
              libboost-all-dev \
              qtbase5-dev qt5-qmake
CMD ["/app/tele"]
