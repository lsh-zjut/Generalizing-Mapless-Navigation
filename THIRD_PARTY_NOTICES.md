# Third-Party Notices

This repository contains original project code together with files derived from or adapted from third-party open-source projects.

## Repository License

The top-level [LICENSE](./LICENSE) file contains the MIT license used for this repository release.

## Included Third-Party Material

### Clearpath Robotics / related BSD-3-Clause material

The repository includes bundled Jackal robot description assets and accessory files under:

- `sim_world/src/jackal_description/`

Some files in that package contain their own BSD-style copyright and redistribution notice in-file, including:

- `sim_world/src/jackal_description/urdf/accessories/sick_lms1xx_upright_mount.urdf.xacro`
- `sim_world/src/jackal_description/urdf/accessories/sick_lms1xx_inverted_mount.urdf.xacro`

Those notices must be retained when redistributing the files.

## Practical Publishing Note

Because this repository includes material from multiple sources, treat it as a mixed-license repository:

- top-level project code: MIT license at the repository root
- bundled third-party robot-description assets: keep their original in-file notices
- external ROS and Python dependencies: keep their own licenses as usual
