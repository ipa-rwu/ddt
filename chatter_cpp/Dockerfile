# syntax=docker/dockerfile:experimental
ARG SUFFIX=
ARG BUILDER_SUFFIX=:ros2
ARG PREFIX=rachelwu/
FROM ${PREFIX}ros:galactic as base
FROM ${PREFIX}builder${BUILDER_SUFFIX} as builder

FROM base as build
COPY . /root/ws/src/targets/
RUN --mount=type=bind,from=builder,target=/builder \
    apt-get update -qq && \
    apt-get upgrade -qq && \
    /builder/workspace.bash builder_setup && \
    /builder/workspace.bash build_workspace /root/ws && \
    rm -rf /var/lib/apt/lists/*

FROM build as test
RUN --mount=type=bind,from=builder,target=/builder \
    apt-get update -qq && \
    /builder/workspace.bash test_workspace /root/ws && \
    rm -rf /var/lib/apt/lists/*

# FROM build as install
# RUN --mount=type=bind,from=builder,target=/builder \
#     apt-get update -qq && \
#     /builder/workspace.bash install_workspace /root/ws && \
#     rm -rf /var/lib/apt/lists/*

# FROM install as full
# RUN --mount=type=bind,from=builder,target=/builder \
#     apt-get update -qq && \
#     /builder/workspace.bash install_depends /root/ws && \
#     rm -rf /var/lib/apt/lists/*

# FROM base as deploy
# RUN --mount=type=bind,from=builder,target=/builder --mount=type=bind,target=/root/ws,from=install,source=/root/ws \
#     apt-get update -qq && \
#     /builder/workspace.bash install_depends /root/ws && \
#     rm -rf /var/lib/apt/lists/*
# COPY --from=install /opt/ros/$ROS_DISTRO /opt/ros/$ROS_DISTRO

FROM build as deploy
ENV SETUP=/root/ws/install/setup.sh
CMD ["ros2", "launch", "chatter_cpp", "talker.launch.py"]
