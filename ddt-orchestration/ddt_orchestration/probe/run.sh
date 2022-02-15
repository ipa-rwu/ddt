#! /bin/sh
SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
app=$SCRIPTPATH/probe.py
$app -a chatter-galatic -p chatter-takler -ip 127.0.0.1 -d 42
