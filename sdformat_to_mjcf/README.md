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

Install the `sdformat_to_mjcf` and `sdformat_mjcf_utils` packages in "editable" mode

```
pip install -e path/to/sdformat_to_mjcf
pip install -e path/to/sdformat_mjcf_utils
```

To convert an SDFormat file to mjcf:

```
python -m sdformat_to_mjcf.sdformat2mjcf path/to/file.sdf | tee new_file.xml
```

To run the mjcf file in Mujoco, download a Mujoco release from https://github.com/deepmind/mujoco/releases, extract the contents and run

```
<path/to/mujoco>/bin/simulate new_file.xml
```

### Running tests


To run tests, either run the test files individually, eg:

```
python tests/test_add_geometry.py
```

or run `pytest` from the root directory.
