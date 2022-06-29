# sdformat-mjcf

This Python package allows bidirectional conversion between SDFormat and MJCF
to share worlds and robot models. We have created a command line tool to convert
between these two formats. It takes as input an SDFormat file that works in
Gazebo Sim and produces as output a MJCF file that works in Mujoco with
approximately equivalent results; and vice versa.

## Install sdformat-mjcf

To start development, create a python3 virtual environment, upgrade pip and
install dm-control

```bash
python3 -m venv path/to/venv --system-site-packages
. path/to/venv/bin/activate

pip install -U pip
pip install dm-control
```

Install `python3-ignition-math7` and `python3-sdformat13` from the
[nightly](https://gazebosim.org/docs/all/release#type-of-releases) repo.

Install the `sdformat-mjcf` package

```bash
pip install sdformat-mjcf
```

### Running tests

Simple run of test can be done by using:

```
python -m unittest
```

# Tools for converting SDFormat to MJCF

Use the commnad line tool `sdf2mjcf`:

```bash
usage: sdf2mjcf [-h] input_file output_file

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
    - Ball
    - Continuous
    - Fixed
    - Prismatic
    - Revolute
  - Materials

## Unsuported features

  - Nested models
  - Links with multiple parents and kinematic loops
  - Revolute2 and Universal joints
  - `<scene>` element
  - `<physics>` element
  - Models from [Fuel](https://app.gazebosim.org/dashboard)
  - Models contain URIs with schemes such as `model://` or `package://`.

## Other limitations:

  - Collada (`.dae`) meshes are not supported by Mujoco. Therefore, the user
    has to first convert each `.dae` file to `.obj` or `.stl` file using
    available tools such as `blender` or `meshlab`. The SDFormat file has to
    then be updated to point to the converted mesh files instead of the `.dae`
    files.
  - Mujoco does not support composite `.obj` files. However, users may process
    the output `.obj` files with
    [obj2mjcf](https://github.com/kevinzakka/obj2mjcf) to split them into
    individual `.obj` files.
  - Only the diffuse texture from a PBR material is converted to MJCF. Other
    textures are not supported.

# Tools for converting MJCF to SDFormat

Use the commnad line tool `mjcf2sdf`:

```bash
usage: mjcf2sdf [-h] [--export_world_plugins] input_file output_file

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
    - Fixed
    - Free
    - Hinge
    - Slide
  - Materials

## Unsuported features

  - Tendon
  - Generation of procedural textures
  - Actuators
  - Equality Constraints
  - Collision filters
  - hfields
  - skins
  - `option.timestep`
  - Fitting of meshes with primitives, convex hulls or AABBs
  - Settings that affect the constraint solver
  - Lights that track or target objects

## Other limitations:

  - Only the values `xyz` and `XYZ` are supported for `compiler.eulerseq`
  - `local` coordinates are assumed for `compiler.coordinate`
  - Each kinematic tree in `<worldbody>` is placed inside a `<model>` when
    converted to SDFormat. The `<self_collide>` element is always set to false
    for `<model>`s to avoid collisions between links connected by mulitple
    joints in series.

