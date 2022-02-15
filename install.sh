#!/bin/bash

if [ ! -d venv ]; then
    for p in "py -3.7" "py -3" "python3.7" "python3" "python"; do
        if $p -V &> /dev/null; then
            pycmd=$p
            break
        fi
    done
    ${pycmd:?Could not determine python executable} -mvenv venv
fi

venv/*/pip install --find-links wheels -e ros2-model
venv/*/pip install --find-links wheels -e ros2-helpers
venv/*/pip install --find-links wheels -e ddt-orchestration
