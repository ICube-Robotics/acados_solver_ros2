# Copyright 2023 ICUBE Laboratory, University of Strasbourg.
# All rights reserved.
# License: Apache License, Version 2.0

# Author: Thibault Poignonec (tpoignonec@unistra.fr)

import os
import re
from shutil import rmtree

# Files/folders management utils


def ensure_dir_exists(dir_path):
    # Create a new directory if it does not exist
    if not os.path.exists(dir_path):
        try:
            os.makedirs(dir_path)
        except Exception as e:
            print(f'Failed to create the dir {dir_path}. Exception: {e}')
            raise


def delete_dir_recursively(dir_path, keep_empty_folder=True):
    try:
        rmtree(dir_path)
    except Exception as e:
        print(f'Failed to clear {dir_path}. Exception: {e}')
        raise
    # rmtree completely remove the dir, so it has to be recreated
    if keep_empty_folder:
        ensure_dir_exists(dir_path)

# Reformat strings from capital to underscore


_uppercase_part = re.compile('[A-Z][^A-Z]*')


def uppercase_to_underscore(string: str) -> str:
    """
    Reformat a camel case string to snake case.

    :param string: The string to reformat
    :type string: str
    :return: Snake case reformated string
    :rtype: str

    >>> uppercase_to_underscore("TestClass")
    "test_class"
    """
    result = ''
    for match in _uppercase_part.finditer(string):
        if match.span()[0] > 0:
            result += '_'
        result += match.group().lower()
    return result
