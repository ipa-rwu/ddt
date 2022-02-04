#! /bin/sh
SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
app=$SCRIPTPATH/"view.py"
export FLASK_ENV=development
export FLASK_APP=$app
flask run
