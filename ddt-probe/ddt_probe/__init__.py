import click
from ddt_probe.probe import main
from ddt_probe.utils import PodInfo
import os

@click.command()
@click.argument('app_id')
@click.argument('pod_id')
@click.argument('pod_ip')
@click.argument('domain_id')
@click.argument('server_ip')
@click.argument('server_port')
def cli(server_ip, server_port, app_id, pod_id, pod_ip, domain_id):
    podinfo = PodInfo(app_id=app_id,
                        pod_id = pod_id,
                        pod_ip = pod_ip,
                        domain_id = domain_id,
                        debug = os.getenv('ZENOH_DEBUG', None),
                        dst_port = os.getenv('DST_PORT', 7447),
                        listen_port = os.getenv('LISTEN_PORT', 8000),
                        listen_server = os.getenv('LISTEN_SERVER', "0.0.0.0"))
    main(server_ip, server_port, podinfo)
