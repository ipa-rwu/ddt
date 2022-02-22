# syntax=docker/dockerfile:1
FROM python:3.8-slim-buster as base

FROM python:3.8-slim-buster as builder
ARG PKG
RUN apt-get update

RUN python -m venv /app/venv/
# Make sure we use the virtualenv:
ENV PATH=/app/venv/bin:$PATH
WORKDIR /app
RUN mkdir ${PKG}
COPY  . ${PKG}
RUN pip install wheel
RUN pip wheel -w "wheels" --find-links=wheels -e ${PKG}

FROM python:3.8-slim-buster as build-image
COPY --from=builder /app/wheels /app/wheels
ENV PATH=/app/venv/bin:$PATH
# ARG PKG
# WORKDIR /app/${PKG}
# RUN apt-get update && \
#     apt-get clean
