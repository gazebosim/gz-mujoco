# -*- coding: utf-8 -*-

from setuptools import setup, find_packages
import pathlib

here = pathlib.Path(__file__).parent.resolve()
long_description = (here / "README.md").read_text(encoding="utf-8")
license_content = (here / "LICENSE").read_text(encoding="utf-8")


setup(
    name='mjcf_to_sdformat',
    version='0.0.1',
    description='Tool to convert MJCF file to SDFormat',
    long_description=long_description,
    long_description_content_type="text/markdown",
    author='Open Robotics',
    author_email='info@openrobotics.org',
    url='https://github.com/ignitionrobotics/gz-mujoco',
    license=license_content,
    packages=find_packages(exclude=('tests', 'docs')),
    test_suite='tests',
    entry_points={
        'console_scripts': [
            'mjcf2sdformat = mjcf_to_sdformat.cli:main',
        ],
    },
)
