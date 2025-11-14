# sdformat-mjcf

This Python package allows bidirectional conversion between SDFormat and MJCF
to share worlds and robot models. We have created a command line tool to convert
between these two formats. It takes as input an SDFormat file that works in
Gazebo Sim and produces as output a MJCF file that works in Mujoco with
approximately equivalent results; and vice versa.

## Install sdformat-mjcf

To start development, create a python3 virtual environment, and upgrade pip.

```bash
python3 -m venv path/to/venv --system-site-packages
. path/to/venv/bin/activate

pip install -U pip
```

Install `python3-gz-math9` and `python3-sdformat16` from the
[nightly](https://gazebosim.org/docs/all/release#type-of-releases) repo. On macOS, add the [osrf/simulation](https://github.com/osrf/homebrew-simulation) tap and install `sdformat16`:
```
brew tap osrf/simulation
brew install sdformat16
```

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

## Supported features

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

## Missing features
The following is a list of missing features divided into features that are currently unimplemented, but may be
implemented at a later time ("Currently unimplemented") and features that are unlikely to be implemented because they
are currently not supported by the MJCF specification ("Unsupported by MJCF").

The list only contains SDFormat elements that affect the physical properties of a model. Unless otherwise
stated, SDFormat elements that affect the behavior of the simulator (i.e not the physical properties of a model), e.g.,
`<world><physics>`, are not implemented.

## Currently unimplemented features

  - Nested models
  - Links with multiple parents and kinematic loops
  - Revolute2 and Universal joints
  - `<scene>` element in SDFormat
  - `<physics>` element in SDFormat
  - Models from [Fuel](https://app.gazebosim.org/dashboard)
  - Models that contain URIs with schemes such as `model://` or `package://`.
  - Collision bitmasks
  - The `<self_collide>` tag in models and links

## Unsupported by MJCF:

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
  - The following link elements
    - `velocity_decay`
    - `projector`
    - `audio_sink`
    - `audio_source`
    - `battery`
    - `particle_emitter`

# Tools for converting MJCF to SDFormat

Use the command line tool `mjcf2sdf`:

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

## Supported features

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

## Missing features
The following is a list of missing features divided into features that are currently unimplemented, but may be
implemented at a later time ("Currently unimplemented") and features that are unlikely to be implemented because they
are currently not supported by the SDFormat specification ("Unsupported by SDFormat").

The list only contains MJCF elements that affect the physical properties of a model. Unless otherwise
stated, MJCF elements that affect the behavior of the simulator (i.e not the physical properties of a model), e.g.,
`option.timestep`, are not implemented.

### Currently unimplemented

The following is a list of unimplemented features that affect the physical properties of a model. Unless otherwise
stated, MJCF elements that affect the behavior of the simulator (i.e not the physical properties of a model), e.g.,
`option.timestep`, are not implemented.

  - **`<compiler>`**
    - Values for `compiler.coordinate` other than `local`
    - Values for `compiler.eulerseq` other than `xyz` and `XYZ`
    - Fitting of meshes with primitives, convex hulls or AABBs

  - Generation of procedural textures
  - `hfield` geoms
  - Collision bitmasks (`contype`, `conaffinity`)
  - `<composite>` elements.
  - Equality Constraints (`<equality>`)
  - Settings that affect the Mujoco visualizer (`<visual>`)

## Unsupported by SDFormat
  - Skins
  - Contact `pair`s and `exclude`s (`<contact>`)
  - Tendons (`<tendon>`)
  - Actuators (`<actuator>`)
  - Settings that affect the constraint solver
  - Lights that track or target objects
  - Cameras that track or target objects

## Other limitations:

  - Each kinematic tree in `<worldbody>` is placed inside a `<model>` when
    converted to SDFormat. The `<self_collide>` element is always set to false
    for `<model>`s to avoid collisions between links connected by multiple
    joints in series.

