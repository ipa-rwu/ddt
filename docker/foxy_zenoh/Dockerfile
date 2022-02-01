# syntax=docker/dockerfile:experimental
ARG SUFFIX=
ARG BUILDER_SUFFIX=:ros2
ARG PREFIX=rachelwu/
FROM ros:foxy-ros-core as base
FROM ${PREFIX}builder${BUILDER_SUFFIX} as builder

FROM base as build
COPY zenoh-bridge-dds /root/
