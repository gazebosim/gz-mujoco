# -*- coding: utf-8 -*-
from setuptools import setup, find_packages
import pathlib


# Adapted from https://github.com/pypa/pip/blob/main/setup.py#L15 based on the
# guide:
# https://packaging.python.org/en/latest/guides/single-sourcing-package-version/
def get_version(rel_path: str) -> str:
    for line in open(rel_path).read().splitlines():
        if line.startswith("__version__"):
            # __version__ = "0.9"
            delim = '"' if '"' in line else "'"
            return line.split(delim)[1]
    raise RuntimeError("Unable to find version string.")


here = pathlib.Path(__file__).parent.resolve()
long_description = (here / "README.md").read_text(encoding="utf-8")


setup(
    name='sdformat_mjcf',
    version=get_version('src/sdformat_mjcf/__version__.py'),
    description='Tool to convert between SDFormat and MJCF',
    long_description=long_description,
    long_description_content_type="text/markdown",
    author='Open Robotics',
    author_email='info@openrobotics.org',
    url='https://github.com/gazebosim/gz-mujoco',
    license='Apache License 2.0',
    classifiers=[
        'License :: OSI Approved :: Apache Software License',
    ],
    python_requires='>=3.8',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    test_suite='tests',
    entry_points={
        'console_scripts': [
            'sdf2mjcf = sdformat_mjcf.sdformat_to_mjcf.cli:main',
            'mjcf2sdf= sdformat_mjcf.mjcf_to_sdformat.cli:main',
        ],
    },
)
