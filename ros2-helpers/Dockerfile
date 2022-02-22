# syntax=docker/dockerfile:1
ARG BASE_IMAGE
ARG PREFIX
ARG SUFFIX
ARG BASE_SUFFIX
FROM ${PREFIX}${BASE_IMAGE}${SUFFIX} as builder
FROM ${PREFIX}ros${BASE_SUFFIX} as base
COPY --from=builder /app/wheels /app/wheels

RUN apt-get update && apt-get -y install python3 python3-pip python3.8-venv
RUN python3 -m pip install --user --upgrade pip

RUN python3 -m venv /app/venv/
ENV PATH=/app/venv/bin:$PATH
ARG PKG
WORKDIR /app
RUN mkdir ${PKG}
COPY  . ${PKG}
RUN pip install wheel
RUN pip install wheels/*-any.whl
RUN pip wheel -w "wheels" --find-links=wheels -e ${PKG}

COPY --from=builder /app /app
ENV PATH=/app/venv/bin:$PATH