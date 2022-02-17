from shutil import rmtree, move
from pathlib import Path

def save_to_file(result_path, fullname, instance):
    f_path = Path(str(result_path) + f'{fullname}.json')
    f_path.parent.mkdir(parents=True, exist_ok=True)
    with open(f_path, "w+") as f:
        f.write(instance.json(indent=4, sort_keys=True))

def backup(result_path, bk_path):
    if result_path.is_dir():
        if bk_path.is_dir():
            rmtree(bk_path, ignore_errors=True)
        move(src=result_path, dst=bk_path)
    rmtree(result_path, ignore_errors=True)

def backup_to_finial(result_path, bk_path):
    if bk_path.is_dir():
        if result_path.is_dir():
            rmtree(result_path, ignore_errors=True)
        move(src=bk_path, dst=result_path)