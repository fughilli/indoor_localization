# Moved to Splanc

The Splanc tracker demos now live in
[fughilli/splanc](https://github.com/fughilli/splanc) under `tools/vive/demos`:

- the flashlight (`flashlight.py`, `effects/flashlight_pose.glsl`)
- the tracker ↔ fixture calibration wizard and the plane / sphere demo (`app.py`, `--p4`)
- the scavenger hunt (`effects/hot_spots_pose.glsl`)

So does a copy of the pose server they read from (`tools/vive/pose_server`, from
`swarm/tools/vive_web`). Start with Splanc's `docs/guides/vive-tracker-lamp.md`.

They were moved from this directory at `05d9473` (#6's head), with the changes listed in
Splanc's `tools/vive/PROVENANCE.md`. `swarm/tools/vive_web` stays here for the libsurvive
recordings `swarm/tools/vive_diag.py` reads.
