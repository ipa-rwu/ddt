## deploy
```
ka talker_galactic.yaml
ka listener_galactic.yaml

# combine into one file
kubectl kustomize > compiled.yaml
```

## communicate
```
# on listener
RUST_LOG=INFO ./root/zenoh-bridge-dds -d $ROS_DOMAIN_ID -l tcp/0.0.0.0:8000

# on talker
RUST_LOG=DEBUG ./root/zenoh-bridge-dds -d $ROS_DOMAIN_ID -e tcp/service-listener:8000
```
