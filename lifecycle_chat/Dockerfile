# syntax=docker/dockerfile:experimental
ARG SUFFIX
ARG PREFIX
ARG PROBE
FROM ${PREFIX}builder:ros2 as builder
FROM ${PREFIX}${PROBE}${SUFFIX} as base

FROM base as build
COPY . /root/ws/src/targets/
RUN --mount=type=bind,from=builder,target=/builder \
    apt-get update -qq && \
    apt-get upgrade -qq && \
    /builder/workspace.bash builder_setup && \
    /builder/workspace.bash build_workspace /root/ws && \
    rm -rf /var/lib/apt/lists/*

# FROM build as install
# RUN --mount=type=bind,from=builder,target=/builder \
#     apt-get update -qq && \
#     /builder/workspace.bash install_workspace /root/ws && \
#     rm -rf /var/lib/apt/lists/*

# ENV SETUP=/root/ws/install/setup.sh
WORKDIR /app/ddt-probe
CMD ["ros2", "launch", "lifecycle_chat", "scenario1.py"]
