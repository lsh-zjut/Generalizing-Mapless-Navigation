# Third-Party Notices

This repository contains original project code together with files derived from or adapted from third-party open-source projects.

## Repository License

The top-level [LICENSE](./LICENSE) file preserves the MIT license text that accompanied the visual navigation code this repository was derived from.

## Included Third-Party Material

### Clearpath Robotics / related BSD-3-Clause material

The following files contain their own BSD-style copyright and redistribution notice in-file:

- `sim_world/src/jackal_description/urdf/accessories/sick_lms1xx_upright_mount.urdf.xacro`
- `sim_world/src/jackal_description/urdf/accessories/sick_lms1xx_inverted_mount.urdf.xacro`

Those notices must be retained when redistributing the files.

### PyTorch torchvision derived source

The following file is marked as modified from the PyTorch torchvision library:

- `deployment/src/inference/models/modified_mobilenetv2.py`

If you continue distributing that file, keep the attribution comment and make sure your final public release remains compatible with the upstream torchvision license terms.

## Practical Publishing Note

Because this repository includes material from multiple sources, treat it as a mixed-license repository:

- top-level project code: MIT-style notice preserved in `LICENSE`
- specific third-party files: keep their original in-file notices

If you want a fully cleaned commercial/public release later, it would be worth doing one more provenance pass over `sim_world/src/jackal_description/` meshes and URDF assets before broad redistribution.
