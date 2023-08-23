import os
from shutil import rmtree

# Files/folders management utils


def ensure_dir_exists(dir_path):
    # Create a new directory if it does not exist
    if not os.path.exists(dir_path):
        try:
            os.makedirs(dir_path)
        except Exception as e:
            print('Failed to create the dir %s. Exception: %s' % (dir_path, e))
            raise


def delete_dir_recursively(dir_path, keep_empty_folder=True):
    try:
        rmtree(dir_path)
    except Exception as e:
        print('Failed to clear %s. Exception: %s' % (dir_path, e))
        raise
    # rmtree completly remove the dir, so it has to be recreated
    if keep_empty_folder:
        ensure_dir_exists(dir_path)
