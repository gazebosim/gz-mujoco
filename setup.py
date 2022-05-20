# -*- coding: utf-8 -*-

from setuptools import setup, find_namespace_packages
import pathlib

here = pathlib.Path(__file__).parent.resolve()
long_description = (here / 'README.md').read_text(encoding='utf-8')
license_content = (here / 'LICENSE').read_text(encoding='utf-8')

# Native namespace package is being used
# https://packaging.python.org/en/latest/guides/packaging-namespace-packages/#native-namespace-packages
packages = find_namespace_packages(
    include=['sdformat_mjcf.*'],
    exclude=['sdformat_mjcf.sdformat_to_mjcf.tests',
             'sdformat_mjcf.sdformat_to_mjcf.tests*',
             'sdformat_mjcf.sdformat_mjcf_utils.tests',
             'sdformat_mjcf.sdformat_mjcf_utils.tests*'])

setup(
    name='sdformat_mjcf',
    version='0.0.1',
    description='Tools to convert between SDFormat and MJCF',
    long_description=long_description,
    long_description_content_type='text/markdown',
    author='Open Robotics',
    author_email='info@openrobotics.org',
    url='https://github.com/gazebosim/gz-mujoco',
    license=license_content,
    packages=packages,
    entry_points={
        'console_scripts': [
            'sdformat2mjcf = sdformat_mjcf.sdformat_to_mjcf.cli:main',
        ],
    },
)
