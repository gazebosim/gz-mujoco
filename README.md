# GZ-MUJOCO

This Python package allows bidirectional conversion between SDFormat and MJCF
to share worlds and robot models. We have created a command line tool to convert
between these two formats. It takes as input an SDF file that works in Gazebo Sim
and produces as output a MJCF file that works in Mujoco with approximately
equivalent results; and vice versa.

# SDF to MJCF

Use the commnad line tool `sdformat2mjcf`:

```bash
usage: sdformat2mjcf [-h] input_file output_file

positional arguments:
  input_file   Path to input SDFormat file (World or Model)
  output_file  Desired path for the output MJCF file

optional arguments:
  -h, --help   show this help message and exit
```

Then you can run the Mujoco simulator with the `output_file`

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


# MJCF to SDF

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

Then you can run the Gazebo Sim simulator with the `output_file`

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

 - Tensor
 - Generation of procedural textures is not supported.
