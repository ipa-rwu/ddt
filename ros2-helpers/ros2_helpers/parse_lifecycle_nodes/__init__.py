from time import sleep
import click
from pathlib import Path
from ros2_helpers.parse_lifecycle_nodes.parse_lifecycle import main
from ros2_helpers.utils import backup, backup_to_finial

@click.command("lifecycle")
@click.option('--result_folder', '-f', type=click.Path(path_type=Path),  default=Path.home() / 'tmp' / 'lifecycle_parser')
@click.option('--rate', '-r', default=1)
def parse_lifecycle(result_folder, rate):
    result_path= result_folder.resolve()
    click.echo(f'lifecycle dst: {result_path}')
    bk_path = result_path.parent / f'{result_path.name}_bk'
    try:
        while True:
            backup(result_path, bk_path)
            main(result_folder)
            sleep(rate)
    except KeyboardInterrupt:
        backup_to_finial(result_path, bk_path)