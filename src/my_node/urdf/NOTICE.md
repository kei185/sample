This URDF is adapted from the Nav2 first-time robot setup tutorial's
`sam_bot_description/src/description/sam_bot_description.urdf`.

- Upstream: https://github.com/ros-navigation/navigation2_tutorials/blob/f3bd892a60d3fb91560ecb726bf922eb9bcbc858/sam_bot_description/src/description/sam_bot_description.urdf
- Revision: `f3bd892a60d3fb91560ecb726bf922eb9bcbc858` (`jazzy`)
- Upstream package maintainer: Josen Daniel De Leon, Samsung
- License: Apache License, Version 2.0, as declared in the upstream package.xml.
  The license text is included in `LICENSE`.

Changes: replace tutorial dimensions with DEVICE.md measurements, place base_link
at the drive axle, replace wheel offsets with the measured wheel separation,
adjust the sensor and caster transforms, remove the unused camera, and remove
unmeasured inertial values. Body, caster, and sensor shapes are approximations for
visualization; this model does not configure a Gazebo dynamics simulation.
