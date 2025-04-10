import os
import sys

# Navigate up two directory levels
sys.path.insert(0, os.path.abspath(".."))
sys.path.insert(0, os.path.abspath("../ros_acomms/src"))
sys.path.insert(0, os.path.abspath("../ros_acomms_modeling/src"))
sys.path.insert(0, os.path.abspath("../ros_acomms_tests/src"))
sys.path.insert(0, os.path.abspath("../ros_acomms_msgs/msg"))
sys.path.insert(0, os.path.abspath("../../../devel/lib/python3/dist-packages"))

# for dirpath, dirnames, filenames in os.walk(
#    os.path.abspath("../../../devel/lib/python3/dist-packages/ros_acomms")
# ):
#   print(dirpath)
# sys.path.insert(0, dirpath)
# print(sys.path)
# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "ros_acomms"
copyright = "2023, Woods Hole Oceanographic Institution"
author = "Woods Hole Oceanographic Institution — Acomms Group"
release = "latest"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

# Napoleon allows using numpy- or Google-style docstrings
extensions = ["myst_parser", "sphinx.ext.autodoc", "sphinx.ext.napoleon", "sphinx_rosmsgs"]

exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

# -- Options for LaTeX output ------------------------------------------------
# Disable index page
latex_elements = {
    "makeindex": "",
    "printindex": "",
}

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output
html_theme = "sphinx_book_theme"
add_function_parentheses = True
add_module_names = True
rosmsg_path_root = ["../ros_acomms_msgs"]
