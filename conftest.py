"""
@file __init__.py / path_config.py
@brief Environment configuration for local module resolution.
"""

import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))