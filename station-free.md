Feature: Support station-free and free-associating localization

North star:
Swarms of widgets can localize off of one another. Any widget that knows its absolute position (such as having a GPS lock) can provide it to the network, enabling every widget to have fast-updating real-time positioning with cm-level precision in any environment.

Widget hardware:
- Each widget has:
  - a 9-axis IMU (acc, gyro, mag)
  - a UWB radio supporting ranging and AoA
  - WiFi
  - BLE
  - (optionally) GPS, or some other absolute positioning mechanism
  - (optionally) altimeter
  - knowledge of the relative poses of its various components (e.g. IMU reference frame W.R.T. UWB antenna orientation)

Each widget participates in an ephemeral network of UWB (for ranging and AoA) + uses WiFi/BLE for backhaul (high-bandwidth data). UWB may also be used for data transmission, but the data payloads should be kept as short as possible so as to not congest the medium.

Each widget measures:
- distance to surrounding widgets
- AoA to surrounding widgets
- IMU
- (if supported) altimeter and GPS

Each widget estimates:
- local pose using kalman filter

Each widget advertises to the rest of the network:
- local pose estimate + current 3D velocity

The algorithms for pose estimation must run entirely on device. The goal is to have the local pose estimates converge on the absolute poses of the widgets after a few iterations. The IMU provides information about fast pose updates, and the UWB measurements are used to squash error relative to other widget poses; the absolute positioning inputs (GPS, altimeter, etc) squash the absolute error for the swarm.

All of these measurements have some characteristic noise/drift (e.g. multipath for UWB, Gyro drift, accelerometer measurement error, double-integration blowing up accelerometer quantization error, etc). This will have to be accounted for in the Kalman filter construction.

For this project, develop a simulation model (model each widget with associated sensors, model sensor accuracy/noise/drift/etc, model widget pose changes over time). Use this simulation model to hill climb the algorithm development. Include a visualization frontend that runs in a browser so that we can visually validate the system performance.

Construct a design document which specifies the full system architecture, including things like protocol design (payloads, transmission rates, sequence diagrams, etc) as well as system requirements (minimum sensor precision, minimum update rate, etc). These should be grounded in experimental results, wherever possible.

Once all of this is done, we will need to port the implementation to the embedded system. To ease porting, do the algorithm development in the target languages (rust and/or C++) and bind it into the simulation workspace through FFI (if using Python) or WASM (if using typescript).

Manage all with bazel + nix.
