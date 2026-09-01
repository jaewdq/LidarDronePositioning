# grid_sensor_calibration

2D rigid-body calibration of ground sensors against a surveyed field grid, and
publication of the resulting `grid -> sensorN` transforms to TF2.

ROS 2 Humble / Jazzy, pure `rclpy` + `numpy` + standard messages.

---

## 1. Problem

A 24 m x 15 m grid is marked on the field with 3 m spacing. Its lower-left
corner is the origin of the `grid` frame (`+x` right, `+y` up, yaw CCW, metres
and radians), so the field covers `0 <= x <= 24`, `0 <= y <= 15`.

Two (or more) 360-degree 2D sensors observe targets and report them in their own
sensor frame. Their design positions are

| sensor    | nominal x | nominal y | nominal yaw |
|-----------|-----------|-----------|-------------|
| `sensor1` | 12.0 m    | -1.0 m    | 0 rad       |
| `sensor2` | 12.0 m    | 16.0 m    | 0 rad       |

but each is installed with an unknown error `(dx, dy, dyaw)`. Calibration
recovers the true pose `(tx, ty, yaw)` of every sensor in the `grid` frame from
targets whose grid coordinates are known, using known correspondences
`Si <-> Gi`. Each sensor is calibrated independently.

## 2. Coordinate frames

```
              y
              ^
              |
   (0,15) +-----------------------------+ (24,15)
          |    .    .    .    .    .    |
          |    .    .    .    .    .    |          sensor2 (12, 16)
          |    .    G4        G3   .    |             ^ y_s2
          |    .    .    .    .    .    |             |
          |    .    .    .    .    .    |     x_s2 <--+     (yaw2 ~ 180 deg
          |    .    G1        G2   .    |                     in this sketch)
   (0,0)  +-----------------------------+ (24,0) --> x
                                                      grid frame origin (0,0)
             sensor1 (12, -1)
                ^ y_s1
                |
                +---> x_s1        (yaw1 ~ 0)

   TF tree                grid
                            |
                   +--------+--------+
                   |                 |
                sensor1           sensor2
```

Convention used everywhere in this package (`calibration_math.py` is the single
source of truth):

```
Pg = R(yaw) * Ps + t          R(yaw) = [[cos, -sin], [sin, cos]],  t = [tx, ty]

Xg = tx + xs*cos(yaw) - ys*sin(yaw)
Yg = ty + xs*sin(yaw) + ys*cos(yaw)
```

* `(tx, ty)` is the **sensor origin expressed in the grid frame**.
* `yaw` is the rotation of the **sensor x-axis relative to the grid x-axis**.
* The inverse is `Ps = R(yaw)^T (Pg - t)`.

That is exactly a TF with `header.frame_id = grid` (parent) and
`child_frame_id = sensorN` (child): transforming a `PointStamped` stamped in
`sensorN` into `grid` through tf2 yields `R Ps + t`.
`test/test_transform.py` asserts this numerically, including the specification's
sanity case: sensor at `(10, 5, 90 deg)`, sensor point `(1, 0)` -> grid
`(10, 6)`.

## 3. Calibration algorithm

```
sensor measurements Si  +  known grid points Gi
              |
              v  centroids          S_bar = mean(Si),  G_bar = mean(Gi)
              v  centred coords     ui = Si - S_bar,   vi = Gi - G_bar
              v  rotation           C = sum(ui . vi),  D = sum(ui x vi)
              |                     yaw = atan2(D, C)
              v  translation        t = G_bar - R(yaw) * S_bar
              v  residuals          ei = Gi - (R Si + t)  ->  RMS / mean / max
              v  TF                 grid -> sensorN  (z=0, roll=pitch=0)
```

This is the 2D Procrustes/Kabsch problem. Two equivalent estimators are
implemented and cross-checked in the tests:

* `method: closed_form` (default) -- the 2D `atan2(D, C)` solution: branch-free,
  cheap, and cannot produce a reflection.
* `method: svd` -- the general Kabsch route: `H = sum(ui vi^T) = U S V^T`,
  `R = V diag(1, det(V U^T)) U^T`. Kept as an independent check and as the
  natural entry point for a future 3D extension.

Minimum `N = 2` (a single target cannot separate yaw from translation);
`N >= 4`, spread across the field, is recommended -- e.g. `(3,3)`, `(21,3)`,
`(21,12)`, `(3,12)`.

**Baseline matters.** For a baseline `L` and per-point noise `sigma`, the yaw
uncertainty is of order `sigma_yaw ~ sqrt(2) * sigma / L`. The simulation
reproduces this within a few percent, so doubling the baseline halves the yaw
error. Also note that with `N = 2` the residual is *not* a quality measure: a
3-DOF transform absorbs almost everything two points can show.

## 4. Package layout

```
grid_sensor_calibration/
├── package.xml
├── setup.py / setup.cfg / pytest.ini
├── README.md
├── resource/grid_sensor_calibration
├── grid_sensor_calibration/
│   ├── calibration_math.py        # pure math: estimation, transforms, validation
│   ├── calibration_io.py          # YAML input/output, dataclasses
│   ├── report.py                  # human-readable calibration report
│   ├── sensor_adapter.py          # driver message -> internal Measurement
│   ├── sensor_calibration_node.py # per-sensor calibration node
│   ├── sensor_tf_publisher.py     # static grid -> sensorN TFs
│   ├── point_transform_node.py    # live detections -> grid frame
│   ├── calibration_viz_node.py    # RViz markers (optional)
│   └── simulation.py              # noise / point-count / baseline studies
├── config/
│   ├── sensor1.yaml               # calibration dataset (method A)
│   ├── sensor2.yaml
│   ├── calibration_params.yaml    # parameters of the calibration nodes
│   ├── runtime_params.yaml        # parameters of the runtime nodes
│   └── sensors_calibration.yaml   # calibration result (combined layout)
├── launch/
│   ├── calibration.launch.py
│   └── sensor_tf.launch.py
├── rviz/grid_calibration.rviz
└── test/
    ├── test_calibration_math.py   # math, validation, sign conventions
    ├── test_calibration_io.py     # YAML round trips
    ├── test_transform.py          # tf2 == direct matrix computation
    ├── test_nodes.py              # nodes, static TF round trip, adapters
    └── test_simulation.py         # synthetic recovery, noise, baseline law
```

Nothing hard-codes "two sensors": the sensor list is a launch argument, the
result file holds one entry per sensor, and adding `sensor3` means adding
`config/sensor3.yaml`, a parameter block, and the name to `sensors:=`.

## 5. Build and test

```bash
cd ~/drone_project/DronePositioningSystem/ros2_ws
colcon build --packages-select grid_sensor_calibration --symlink-install
source install/setup.bash

# unit + integration tests (84 tests)
python3 -m pytest src/grid_sensor_calibration -q
```

`colcon test` falls back to the (empty) `setup.py test` runner on this machine,
because the locally installed setuptools drops the `tests_require` field that
colcon inspects. Run `python3 -m pytest` as shown above.

## 6. Run

### Calibration (method A: recorded YAML)

```bash
ros2 launch grid_sensor_calibration calibration.launch.py \
    sensors:=sensor1,sensor2 \
    config_dir:=$HOME/grid_calibration \
    calibration_file:=$HOME/grid_calibration/sensors_calibration.yaml \
    shutdown_after_calibration:=true
```

Each node prints the report of section 27 of the specification (estimated pose,
nominal pose, installation error, RMS/mean/max residual, per-point table) and
writes its own entry into the combined result file.

### Calibration (method B: live capture)

```bash
ros2 run grid_sensor_calibration sensor_calibration_node --ros-args \
    -r __node:=sensor1_calibration \
    -p sensor_name:=sensor1 -p child_frame:=sensor1 \
    -p nominal_x:=12.0 -p nominal_y:=-1.0 \
    -p auto_calibrate:=false \
    -p 'grid_points:=[3.0, 3.0, 21.0, 3.0, 21.0, 12.0, 3.0, 12.0]' \
    -p calibration_file:=$HOME/grid_calibration/sensors_calibration.yaml

# with the sensor publishing /sensor1/target_point, for each target in order:
ros2 service call /sensor1_calibration/capture_point std_srvs/srv/Trigger
# then:
ros2 service call /sensor1_calibration/run_calibration std_srvs/srv/Trigger
# mistake? start over:
ros2 service call /sensor1_calibration/reset std_srvs/srv/Trigger
```

`capture_point` averages the last `capture_sample_count` samples and reports the
sample standard deviation, so a noisy or moving target is visible immediately.

### Runtime (static TF + live point transformation)

```bash
ros2 launch grid_sensor_calibration sensor_tf.launch.py \
    calibration_file:=$HOME/grid_calibration/sensors_calibration.yaml \
    visualization:=true

ros2 run tf2_ros tf2_echo grid sensor1
ros2 topic echo /sensor1/target_point_grid
```

`transform_mode:=direct` bypasses TF2 and applies `Pg = R Ps + t` straight from
the YAML -- useful for replaying bags; the tests assert both modes agree.

### Simulation studies

```bash
ros2 run grid_sensor_calibration calibration_simulation --trials 500
```

## 7. RViz2

```bash
rviz2 -d $(ros2 pkg prefix grid_sensor_calibration)/share/grid_sensor_calibration/rviz/grid_calibration.rviz
```

* **Fixed Frame**: `grid`
* **TF**: shows `grid`, `sensor1`, `sensor2`
* **MarkerArray** `/calibration_viz_node/markers` (transient local): grid lines,
  calibration targets, sensor poses with heading and RMS label, measurements
  re-projected into the grid frame, and residual vectors (exaggerated by
  `residual_scale`).
* **PointStamped** `/sensor1/target_point_grid`, `/sensor2/target_point_grid`.

## 8. Field procedure

1. Place the targets, ideally 4 or more spread over the whole grid
   (`(3,3)`, `(21,3)`, `(21,12)`, `(3,12)`).
2. Survey each target in the grid frame and write the values into
   `grid_points` of `config/sensorN.yaml`.
3. Record sensor 1's reading of every target, in the same order, into
   `sensor_points`.
4. Repeat for sensor 2 (own file, own order is irrelevant as long as it matches
   its own `grid_points`).
5. Run `calibration.launch.py`.
6. Check the residuals: RMS should be at the level of the sensor noise. A single
   large `|e|` in the per-point table means a swapped correspondence or a moved
   target.
7. The result YAML is written automatically; keep it under version control.
8. Start `sensor_tf.launch.py` -- the static TFs come from the file, no
   recalibration.
9. Feed real detections into `/sensorN/target_point`.
10. Verify `/sensorN/target_point_grid`: both sensors must report the *same*
    grid coordinates for one physical target.

## 9. Interfaces: now and later

**Current (minimal, standard messages only)**

| purpose                  | interface                                            |
|--------------------------|------------------------------------------------------|
| sensor detection in      | `geometry_msgs/PointStamped` (via adapter)           |
| detection in grid frame  | `geometry_msgs/PointStamped` on `..._grid` + fused   |
| calibration control      | `std_srvs/Trigger` x3 (capture / calibrate / reset)  |
| calibration result out   | `geometry_msgs/TransformStamped` (latched) + YAML    |
| sensor pose              | TF2 static `grid -> sensorN`                         |
| visualisation            | `visualization_msgs/MarkerArray`                     |

**Recommended later**, once the sensor drivers are fixed:

* `TargetDetection.msg` (`Header`, `geometry_msgs/Point`, `float64 intensity`,
  `float64[4] covariance`, `uint32 id`) and `TargetDetectionArray.msg` -- a
  sensor usually sees several candidates per scan, and fusion needs covariance
  and a real timestamp per detection. `GridDetection` in
  `point_transform_node.py` is already the internal equivalent.
* `CapturePoint.srv` (request: target index / expected grid position; response:
  captured measurement, sample count, standard deviation) to replace the
  ordered `Trigger` capture.
* `Calibrate.srv` (response: pose, residuals, installation error) so a GUI can
  show the result without parsing the log.

The `sensor_adapter.py` layer exists so the driver message type can change
without touching node logic: add an adapter class, set `input_message_type`.

## 10. Extension points

* **More sensors** -- configuration only, as described in section 4.
* **Association / fusion / tracking** -- every detection is published in the
  common `grid` frame on `fused_topic` and passed to
  `PointTransformNode.on_grid_detection()`. A nearest-neighbour associator, a
  Kalman/EKF tracker or a velocity estimator attaches there, downstream of all
  frame handling.
* **Timestamp synchronisation** -- `Measurement.stamp` carries the original
  sensor stamp through the transformation, so a future fusion node can use
  `message_filters` without changes here.
* **3D** -- `calibration_math.py` is written so the SVD path generalises to
  3x3 rotations; the TF messages are already 3D with `z = 0`.
