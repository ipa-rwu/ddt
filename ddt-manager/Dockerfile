# syntax=docker/dockerfile:1
ARG PREFIX
ARG SUFFIX
ARG BASE_IMAGE
FROM ${PREFIX}${BASE_IMAGE}${SUFFIX} as builder

FROM python:3.8-slim-buster as base
COPY --from=builder /app/wheels /app/wheels

ARG PKG
RUN apt-get update
RUN apt-get install -y --no-install-recommends build-essential gcc graphviz graphviz-dev wget && \
    rm -rf /var/lib/apt/lists/*

RUN python -m venv /app/venv/
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
CMD ["./run.sh"]