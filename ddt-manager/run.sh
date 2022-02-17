#! /bin/sh
SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
app=$SCRIPTPATH/"ddt_manager/manager/view.py"
export FLASK_ENV=development
export FLASK_APP=$app
flask run --host=0.0.0.0 --port=1234
