FROM gcc:latest

WORKDIR /home

RUN apt update \
 && apt install -y make \
                   cmake \
                   gdb \
                   git \
                   wget \
                   tar \
                   zip \