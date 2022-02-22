# syntax=docker/dockerfile:experimental
ARG BUILDER_SUFFIX
ARG PREFIX
ARG SUFFIX
FROM ${PREFIX}ros${SUFFIX} as base
FROM ${PREFIX}builder${BUILDER_SUFFIX} as builder

FROM base as build
COPY . /root/ws/src/targets/
RUN --mount=type=bind,from=builder,target=/builder \
    apt-get update -qq && \
    apt-get upgrade -qq && \
    /builder/workspace.bash builder_setup && \
    /builder/workspace.bash build_workspace /root/ws && \
    rm -rf /var/lib/apt/lists/*

FROM build as deploy
RUN --mount=type=bind,from=builder,target=/builder \
    apt-get update -qq && \
    apt-get upgrade -qq && \
    /builder/workspace.bash install_depends /root/ws && \
    rm -rf /var/lib/apt/lists/*

ENV SETUP=/root/ws/install/setup.sh
CMD ["ros2", "launch", "ros2_graph_quest", "gen_dot.launch.py", "sampling_rate:=1"]
