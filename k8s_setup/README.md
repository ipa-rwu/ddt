#DDT

## Setup k8s

### Setup master

0. cd k8s_setup/builder

1. check ip in temp_config.yaml

2. check cluster name

3. `./setup_master.sh ddt temp_config.yaml`

4. apply flannel
`kubectl apply -f https://raw.githubusercontent.com/coreos/flannel/master/Documentation/kube-flannel.yml`

5. check `ip route`, update "macvlan-interface.yaml", the apply

6. join token
`kubeadm token create --print-join-command`

7. use dns
```
cd coredns
$ ./deploy.sh | kubectl apply -f -
```

8. set up [local regristry](../docker/registry/README.md)

## setup worker
0. `cd k8s_setup/builder`
1. `./setup_worker.sh <kubernetes cluster master address> <discovery token>`
``
    echo "Usage: $1 [<kubernetes cluster master address>]" >&2
    echo "Usage: $2 [<token>]" >&2
    echo "Usage: $3 [<discovery token>]" >&2

##setup network
### create bridge
```
# create bridge
sudo brctl addbr br0

sudo ip addr add 10.0.0.1/24 dev rosmc

```
### bring up
```
sudo ip link set up rosmc
```
