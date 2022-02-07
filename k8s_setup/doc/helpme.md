# Troubleshoot

1. dial tcp 127.0.0.1:10248: connect: connection refused.

solution: The problem was cgroup driver. Kubernetes cgroup driver was set to systems but docker was set to systemd.
* create "/etc/docker/daemon.json"
```
{
"exec-opts": ["native.cgroupdriver=systemd"]
}
```
* `sudo systemctl daemon-reload`
* `sudo systemctl restart docker`
* `sudo systemctl restart kubelet`

2. clean up master
   * remove worker: `kubectl drain <node name>`
   * power down the node: `kubectl uncordon <node name>`
   * kubectl config delete-cluster "name"
   * sudo kubeadm reset
   * rm $HOME/.kube/config
   * https://kubernetes.io/docs/tasks/administer-cluster/safely-drain-node/


3. check port
   * sudo netstat -lpn |grep :10250

4. kill the process using the port
   * sudo kill -9 `sudo lsof -t -i:9001`

5. ssh Too many authentication failures
   *  ssh -o PubkeyAuthentication=no nuc@192.168.0.115

6. check logs
   * journalctl -u kubelet | tail -n 300
   * journalctl -u kubelet -n 100

7. allow the master to run workloads
   * kubectl taint nodes --all node-role.kubernetes.io/master-

8. Create a token:
   ```
   kubeadm token create
   ```

9. create ca-cert-hash
   ```
   openssl x509 -pubkey -in /etc/kubernetes/pki/ca.crt | openssl rsa -pubin -outform der 2>/dev/null | \
      openssl dgst -sha256 -hex | sed 's/^.* //'
   ```
10. copy docker image
    `docker save <image> | bzip2 | pv | ssh user@host docker load`

11. get join command
   `kubeadm token create --print-join-command`

12. remove pods
   `kubectl drain pod podname`

13. more info
   `kubectl get pods --output=wide`

14. Get a shell to the running container
   `kubectl exec --stdin --tty shell-demo -- /bin/bash`

15. see a specific port such as 22
   `sudo lsof -i:22 `

16. use macvlan
   ```yaml
   apiVersion: "k8s.cni.cncf.io/v1"
   kind: NetworkAttachmentDefinition
   metadata:
   name: ros2-network
   spec:
   config: '{
         "cniVersion": "0.3.0",
         "type": "macvlan",
         "master": "wlan0", # all nodes need to have same interface
         "mode": "bridge",
         "ipam": {
         "type": "host-local",
         "subnet": "192.168.0.0/24", # get from ip route
         "rangeStart": "192.168.0.80",
         "rangeEnd": "192.168.0.110",
         "routes": [
            { "dst": "0.0.0.0/0" }
         ],
         "gateway": "192.168.0.1"
         }
      }'
   ```

17. rename interface
   `sudo gedit /etc/systemd/network/10-uplink.link`
   ```
   [Match]
   MACAddress=70:1c:e7:67:2a:05

   [Link]
   Name=wlan0
   ```
   `lshw -C network`

18. use wireshark
 `kubectl sniff zenoh-router-594d496ff5-g7cnt -v -n default -p  | wireshark -k -i -`
