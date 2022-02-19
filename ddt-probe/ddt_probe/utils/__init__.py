class PodInfo:
    def __init__(self, app_id, pod_id, pod_ip, domain_id, debug, listen_port, listen_server, dst_port):
        self.app_id = app_id
        self.pod_id = pod_id
        self.pod_ip = pod_ip
        self.domain_id = domain_id
        self.debug = debug
        self.listen_port = listen_port
        self.listen_server = listen_server
        self.dst_port = dst_port
