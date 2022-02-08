## Setup Registry
docker-compose -f "docker-compose.yaml" up -d --build

### create certificate
```
mkdir registry_certs

openssl req -newkey rsa:4096 -nodes -sha256 \
-keyout registry_certs/domain.key -x509 \
-out registry_certs/domain.cert
```

### setup client
```
sudo mkdir -p /etc/docker/certs.d/(server):5000

# declear host
/etc/hosts
ip server_name

# copy certificate
registry_certs/domain.cert /etc/docker/certs.d/(server):5000/ca.crt

#config docker
/etc/docker/daemon.json

# add
"insecure-registries" : [ "(server):5000" ]
```
