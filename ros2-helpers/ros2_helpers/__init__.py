import click

from ros2_helpers.parse_lifecycle_nodes import parse_lifecycle as parse_lifecycle_command
from ros2_helpers.parse_nodes import parse_nodes as parse_nodes_command

@click.group()
def cli():
    pass

cli.add_command(parse_lifecycle_command)
cli.add_command(parse_nodes_command)
