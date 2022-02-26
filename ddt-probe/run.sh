#! /bin/sh
# ddt_probe chatter-galatic chatter-takler  127.0.0.1 42 127.0.0.1 1234
ddt_probe ${APP_ID} ${POD_ID}  ${POD_IP} ${ROS_DOMAIN_ID} ${DDT_MANAGER_SERVER} ${DDT_MANAGER_PORT} &
python3 -m http.server ${DDT_PROBE_SERVER_PORT} --bind ${DDT_PROBE_SERVER} --directory ${DDT_FOLDER}