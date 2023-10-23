# Copyright 2022 ICube Laboratory, University of Strasbourg
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
from pathlib import Path
import subprocess

# Doxygen
subprocess.call('cd ../doxygen; doxygen Doxyfile', shell=True)
html_extra_path = ['../doxygen/_build']  # Export files to "sphinx/_build"

# -- Project information -----------------------------------------------------

project = "acados_controllers_ros2"
copyright = "2023, ICUBE Laboratory, University of Strasbourg"
author = "Thibault Poignonec"

# The full version, including alpha/beta/rc tags
release = "0.1.0"


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    "sphinx_rtd_theme",
    "sphinx_mdinclude",
    "sphinx.ext.imgmath",
    "sphinx.ext.todo",
    'sphinx.ext.viewcode',
    'sphinx.ext.autodoc',
    "sphinx.ext.graphviz",
    'sphinx.ext.inheritance_diagram',
    "sphinxcontrib.plantuml",
    "breathe",
]

breathe_default_project = "acados_controllers_ros2"


def get_package(package: str):
    path = Path(__file__).parent.parent.parent.joinpath(
        f"{package}/include/{package}"
    )
    files_gen = path.glob("*.hpp")
    files = []
    for file in files_gen:
        files.append(file.name)
    return (path, files)


templates_path = ["_templates"]
primary_domain = "cpp"
highlight_language = "cpp"

breathe_projects = {
    "acados_controllers_ros2": "_build/xml/",
}
breathe_default_project = "acados_controllers_ros2"
breathe_default_members = ('members', 'undoc-members')


# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]


# -- Options for HTML output -------------------------------------------------

html_theme = "sphinx_rtd_theme"  # "sphinx_book_theme"
html_theme_options = {
    'canonical_url': '',
    'analytics_id': '',  # Provided by Google in your dashboard
    'display_version': True,
    'prev_next_buttons_location': 'bottom',
    'style_external_links': False,

    'logo_only': False,

    # Toc options
    'collapse_navigation': True,
    'sticky_navigation': True,
    'navigation_depth': 4,
    'includehidden': True,
    'titles_only': False
}

html_static_path = ["_static"]
html_logo = "images/logo-icube.png"
html_css_files = ["css/custom.css"]
pygments_style = "sphinx"
