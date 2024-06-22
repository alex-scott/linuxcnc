#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
from setuptools import setup, Extension

__version__ = '0.1.0'

setup(
    name = 'fastcanon',
    version = __version__,

    package_data = {
        '': [ '**.txt', '**.md', '**.py', '**.h', '**.hpp', '**.c', '**.cc' ],
    },

    ext_modules = [
        Extension(
            name = 'fastcanon',
            sources = [
                'source/fastcanon.cc',
            ],
            include_dirs = ['source'],
        )
    ],
)