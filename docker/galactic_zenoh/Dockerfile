# syntax=docker/dockerfile:experimental
FROM ros:galactic as base

FROM base as build
RUN apt-get update -qq && \
    apt-get upgrade -qq && \
    apt install libfmt-dev &&\
    rm -rf /var/lib/apt/lists/*
COPY zenoh-bridge-dds /root/
COPY dds_setting.xml /root/

ARG net_interface=eth0
ENV NETINTERFACE=$net_interface
ENV CYCLONEDDS_URI=/root/dds_setting.xml
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
