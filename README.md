# RDS-26 Surgeon Team — Finger Drake Simulation
ME 472 Robot Design Studio 2026 | Northwestern University

Team: Aileen Cleary, Nicholas Melo, Andrew Pavlovic, Julia Xu, Zach Xu

---

## What This Is

This repo contains our Drake simulation of the Surgeon team's robotic finger. The finger runs in Drake's Meshcat visualizer and includes the full kinematic chain, inertial properties, collision geometries, and joint limits based on our physical design.

---

## Requirements

- Ubuntu 22.04
- Python 3.10.12
- Drake 0.0.20260129

---

## Running the Simulation

Note: the simulation code is on the `simulation` branch.

Clone the repo:
```bash
git clone -b simulation https://github.com/AileenCleary/RDS-26-Surgeon.git
cd RDS-26-Surgeon
```

Open `scripts/test_kinematic.py` and update `PACKAGE_ROOT` to the absolute path of the `models` folder on your machine:
```python
PACKAGE_ROOT = "/your/path/to/RDS-26-Surgeon/models"
```

Then run:
```bash
python3 scripts/test_kinematic.py
```

Open `http://localhost:7000` in a browser when the script starts. Press ▶ in Meshcat to replay the animation.
---

## File Structure

```
finger_drake_sim/
├── models/
│   ├── urdf/
│   │   └── finger.sdf
│   └── meshes/
│       ├── holder.gltf
│       ├── mcp.gltf
│       ├── proximal.gltf
│       ├── middle.gltf
│       └── distal.gltf
├── scripts/
│   └── test_kinematic.py
└── README.md
```

---

## Changes from the Original CAD

Our full OnShape model: https://cad.onshape.com/documents/e02a2941237e297f45c70ae0/w/c73063988841cc9b9220d4ff/e/90231eabf24c3b43db7ff2aa

### Simplification to 5 Sub-Assemblies

The original assembly has many individual components — bearings, pulleys, shafts, and structural links. For simulation we merged these into 5 simplified sub-assemblies by combining each phalanx's structural parts with its bearings and pulleys into a single rigid body:

- **Holder Assembly** — base mount, fixed to world
- **MCP Assembly** — MCP joint housing including splay pulley
- **Proximal Phalanx** — proximal link with MCP flexion pulleys and bearings
- **Middle Phalanx** — middle link with PIP pulleys and bearings
- **Distal Phalanx** — distal link with DIP pulleys and bearings

Each sub-assembly was exported individually from OnShape as a `.gltf` file. Exporting the full assembly was not an option since it produces a single static mesh with no joint structure.

### Coordinate Frame Corrections

Drake automatically applies an R_x(+90°) rotation when loading `.gltf` files to convert from glTF's Y-up convention to Drake's Z-up world frame. Each sub-assembly had a different local axis orientation in OnShape, so additional rotations were needed in each link's visual pose. Holder and MCP both point in the +Y direction and required no extra rotation. Proximal points in the +Z direction and needed roll=−π/2 and yaw=−π/2. Middle points in the −X direction and needed pitch=+π/2 and yaw=−π/2. Distal points in the +Y direction and needed yaw=+π/2.

Joint axis positions were measured in each sub-assembly's local coordinate frame in OnShape using the Measure tool, then converted to Drake world coordinates accounting for the automatic R_x(+90°) rotation. Visual mesh offsets were computed by finding the translation needed to align each sub-assembly's upstream joint axis with the link frame origin.

The Distal Phalanx required special handling — its mesh origin does not coincide with the DIP joint axis, so the link pose was offset to place the DIP axis at the correct world position (0, 0, 0.133 m).

### Mass and Inertia

All mass properties were pulled from OnShape's Mass Properties tool and converted: g→kg, g·mm²→kg·m², with inertia axes remapped to match the SDF coordinate convention (OnShape Y → SDF Z). Center of mass coordinates were transformed from OnShape sub-assembly local frames to SDF link frames.

### Collision

Collision geometry reuses the same mesh files as the visuals with identical pose offsets. Adjacent link pairs are excluded from collision via `plant.set_adjacent_bodies_collision_filters(True)`. A box-shaped ground plane is added programmatically in Python.

### Joint Limits

Joint limits are set based on our mechanical design specifications: MCP flexion, PIP, and DIP are limited to 90° of flexion, and MCP splay is limited to ±10°.

---

## Demo

A short demo video of the current finger simulation:

[Download the demo video](demo.mp4)

## AI Usage

Claude was used to help debug SDF coordinate frame calculations, troubleshoot Drake API issues, and assist with code structure.
