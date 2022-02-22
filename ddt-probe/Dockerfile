# syntax=docker/dockerfile:1
ARG PREFIX
ARG SUFFIX
ARG DDT_UTILS_IMAGE=
ARG HELPER_IMAGE=
ARG GRAPH_IMAGE=

FROM ${PREFIX}${DDT_UTILS_IMAGE}${SUFFIX}    as utils
FROM ${PREFIX}${HELPER_IMAGE}${SUFFIX}       as helper
FROM ${PREFIX}${GRAPH_IMAGE}${SUFFIX}        as base

FROM base as build
COPY --from=utils /app/wheels /app/wheels
COPY --from=helper /app/wheels /app/wheels

ARG PKG
RUN apt-get update
RUN apt-get install -y --no-install-recommends python3-pip python3.8-venv && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /app/venv/
# Make sure we use the virtualenv:
ENV PATH=/app/venv/bin:$PATH
WORKDIR /app
RUN mkdir ${PKG}
COPY  . ${PKG}

RUN pip install wheel
RUN pip install wheels/*-any.whl
RUN pip wheel -w "wheels" --find-links=wheels -e ${PKG}

WORKDIR /app/${PKG}
RUN pip install -e .
CMD [ "./run.sh" ]