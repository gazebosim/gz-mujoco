# gz-mujoco converter tool

## Development

To develop the application, an editable installation is available
using the following command in the root of this repository checkout:

    pip3 install -e .

### Running tests

Simple run of test can be done by using:

    python3 -m unittest

A tox (environment manager) run of tests with multiple python versions:

    python3 -m tox

### Run the application

After the editable install in the first point of this section, the gz-mujoco
script defined in `setup.py` entrypoints:

    gz-mujoco
