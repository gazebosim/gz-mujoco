# Tools for converting between SDFormat and MJCF

To start development, create a python3 virtual environment, upgrade pip and
install dm-control

```
python3 -m venv path/to/venv --system-site-packages
. path/to/venv/bin/activate

pip install -U pip
pip install dm-control
```

Install `python3-ignition-math7` from the
[nightly](https://gazebosim.org/docs/all/release#type-of-releases) repo.

Build `libsdformat` from source from the `ahcorde/python/all` branch and add
the installation path to your `LD_LIBRARY_PATH` and `PYTHONPATH`.

```
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$INSTALL_DIR/lib"
export PYTHONPATH="$PYTHONPATH:$INSTALL_DIR/lib/python"
```

where `$INSTALL_DIR` is the installation directory you used when building
libsdformat.

Install the `sdformat_mjcf` packages in "editable" mode

```
pip install -e path/to/sdformat_mjcf
```

To convert an SDFormat file to mjcf:

```
python -m sdformat_mjcf.sdformat2mjcf path/to/file.sdf | tee new_file.xml
```

To run the mjcf file in Mujco, download a Mujco release from https://github.com/deepmind/mujoco/releases,
extract the contents and run

```
<path/to/mujoco>/bin/simulate new_file.xml
```

To run tests, either run the test files individually, eg:

```
python -m unittest
```

A tox (environment manager) run of tests with multiple python versions:

    python3 -m tox

### Run the application

After the editable install in the first point of this section, the sdformat-mjcf
script defined in `setup.py` entrypoints:

    sdformat-mjcf
