# GZ-MUJOCO

This Python package allows bidirectional conversion between SDFormat and MJCF
to share worlds and robot models. We have created a command line tool to convert
between these two formats. It takes as input an SDF file that works in Gazebo Sim
and produces as output a MJCF file that works in Mujoco with approximately
equivalent results; and vice versa.

## Install gz-mujoco

To start development, create a python3 virtual environment, upgrade pip and
install dm-control

```bash
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

Install the `sdformat-mjcf` package in "editable" mode

```bash
pip install -e path/to/sdformat_mjcf
```

### Running tests

Simple run of test can be done by using:

```
python -m unittest
```

# Tools for converting SDFormat to MJCF

Use the commnad line tool `sdformat2mjcf`:

```bash
usage: sdformat2mjcf [-h] input_file output_file

positional arguments:
  input_file   Path to input SDFormat file (World or Model)
  output_file  Desired path for the output MJCF file

optional arguments:
  -h, --help   show this help message and exit
```

To run the MJCF file in Mujoco, download a Mujoco release from https://github.com/deepmind/mujoco/releases,
extract the contents and run

```bash
<path/to/mujoco>/bin/simulate output_file.xml
```

## Suported features

 - Models/Worlds
 - Links
 - Sensors
   - Altimeter
   - Camera
   - Force torque
   - IMU
 - Joints
   - Fixed
   - Hinge
   - Slide
 - Materials

# Tools for converting MJCF to SDFormat

Use the commnad line tool `mjcf2sdformat`:

```bash
usage: mjcf2sdformat [-h] [--export_world_plugins] input_file output_file

positional arguments:
  input_file            Path to input MJCF file
  output_file           Desired path for the output SDFormat file

optional arguments:
  -h, --help            show this help message and exit
  --export_world_plugins
                        Export world plugins
```

If you are going to use the converted file in Gazebo Sim you should use the flag
`--export_world_plugins` to export some of the plugins that are required to make
the new world work properly in Gazebo Sim.

To run the SDFormat file in GazeboSim, follow [these instructions to install Gazebo Sim](https://gazebosim.org/docs/latest/install)

## Suported features

 - Bodies
 - Geoms
 - Sensors
   - Camera
   - Force torque
   - IMU
 - Joints
   - Ball
   - Continuous
   - Fixed
   - Prismatic
   - Revolute
 - Materials

## Unsuported features

 - Tendon
 - Generation of procedural textures is not supported.
