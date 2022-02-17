import click
from ddt_probe.probe import main
from ddt_probe.utils import PodInfo

@click.command()
@click.argument('app_id')
@click.argument('pod_id')
@click.argument('pod_ip')
@click.argument('domain_id')
@click.argument('server_ip')
@click.argument('server_port')
def cli(server_ip, server_port, app_id, pod_id, pod_ip, domain_id):
    podinfo = PodInfo(app_id, pod_id, pod_ip, domain_id)
    main(server_ip, server_port, podinfo)
