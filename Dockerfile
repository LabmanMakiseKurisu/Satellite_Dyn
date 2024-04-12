FROM ubuntu:20.04

WORKDIR /app

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    cmake \
    gcc \
    g++ \
    git \
    && rm -rf /var/lib/apt/lists/*

COPY . .

RUN git submodule update --init --recursive

RUN mkdir -p build && cd build \
    && cmake .. \
    && make

EXPOSE 9088

# 启动命令
CMD ["./build/release/Satellite"]
