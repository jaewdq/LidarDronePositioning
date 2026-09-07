#!/usr/bin/env python3

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String


# ============================================================================
# DATA STRUCTURES
# ============================================================================

@dataclass
class Sample:
    index: int
    t: float
    x: float
    y: float
    z: float
    speed: float = 0.0


@dataclass
class HoverResult:
    marker_id: int
    detected: bool = False
    detection_time: Optional[float] = None
    detection_index: Optional[int] = None
    visit_end_index: Optional[int] = None
    valid: bool = False
    start_index: Optional[int] = None
    end_index: Optional[int] = None
    start_time: Optional[float] = None
    end_time: Optional[float] = None
    duration: float = 0.0
    rmse: Optional[float] = None
    min_distance: Optional[float] = None


@dataclass
class SegmentResult:
    name: str
    source_id: int
    destination_id: int
    source_xy: Tuple[float, float]
    destination_xy: Tuple[float, float]
    start_index: Optional[int] = None
    end_index: Optional[int] = None
    start_time: Optional[float] = None
    end_time: Optional[float] = None
    completed: bool = False
    rmse: Optional[float] = None
    mean_error: Optional[float] = None
    max_error: Optional[float] = None
    sample_count: int = 0


# ============================================================================
# MATHEMATICAL / OFFLINE-INFERENCE FUNCTIONS
# These intentionally mirror offline_compet_score.py.
# ============================================================================

def rmse(values):
    vals = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    if not vals:
        return None
    return math.sqrt(sum(v * v for v in vals) / len(vals))


def mean_value(values):
    vals = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    if not vals:
        return None
    return sum(vals) / len(vals)


def score(error, limit):
    if error is None or not math.isfinite(error) or limit <= 0.0:
        return 0.0
    return max(0.0, min(100.0, 100.0 * (1.0 - error / limit)))


def tolerance_score(error, full_limit, zero_limit):
    if error is None or not math.isfinite(error):
        return 0.0
    if full_limit < 0.0 or zero_limit <= full_limit:
        return 0.0
    if error <= full_limit:
        return 100.0
    if error >= zero_limit:
        return 0.0
    return 100.0 * (zero_limit - error) / (zero_limit - full_limit)


def point_segment_distance(p, a, b):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    d2 = dx * dx + dy * dy
    if d2 <= 1e-12:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    u = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / d2
    u = max(0.0, min(1.0, u))
    qx = a[0] + u * dx
    qy = a[1] + u * dy
    return math.hypot(p[0] - qx, p[1] - qy)


def nearest_grid_error(x, y, grid_x, grid_y):
    error_x = min(abs(x - gx) for gx in grid_x)
    error_y = min(abs(y - gy) for gy in grid_y)
    return min(error_x, error_y)


def horizontal_distance(sample, target):
    return math.hypot(sample.x - target[0], sample.y - target[1])

def infer_home_from_initial_samples(samples, capture_duration):
    """Estimate run home XY from the initial measured trajectory.

    Uses all valid samples from the first `capture_duration` seconds.
    This lets each run start from an arbitrary takeoff location while
    keeping the final required route 4->3->2->1->home consistent.
    """
    if not samples:
        raise ValueError('Cannot infer home without pose samples.')
    duration = max(0.0, float(capture_duration))
    t0 = samples[0].t
    selected = [s for s in samples if s.t <= t0 + duration + 1e-12]
    if not selected:
        selected = [samples[0]]
    x = sum(s.x for s in selected) / len(selected)
    y = sum(s.y for s in selected) / len(selected)
    return (x, y), len(selected)


def calculate_sample_speeds(samples):
    """Central-difference horizontal speed used for hover inference.

    Vertical motion/noise is deliberately excluded from the hover-speed test.
    """
    if len(samples) < 2:
        if samples:
            samples[0].speed = 0.0
        return

    for i, sample in enumerate(samples):
        if i == 0:
            a, b = samples[0], samples[1]
        elif i == len(samples) - 1:
            a, b = samples[-2], samples[-1]
        else:
            a, b = samples[i - 1], samples[i + 1]

        dt = b.t - a.t
        if dt <= 1e-9:
            sample.speed = 0.0
        else:
            dx = b.x - a.x
            dy = b.y - a.y
            sample.speed = math.sqrt(dx * dx + dy * dy) / dt


def infer_marker_hovers(
    samples,
    markers,
    detection_radius,
    hover_gate_radius,
    hover_speed_threshold,
    required_hover_time,
    hover_zero_score_rmse,
    hover_gap_tolerance,
    max_hover_index=None,
):
    results = {}

    for marker_id in (1, 2, 3, 4):
        target = markers[marker_id]
        distances = [horizontal_distance(sample, target) for sample in samples]

        detected_indices = [
            i for i, distance in enumerate(distances)
            if distance <= detection_radius
        ]
        detected = bool(detected_indices)
        detection_index = detected_indices[0] if detected else None
        detection_time = samples[detection_index].t if detected else None
        minimum_distance = min(distances) if distances else None

        # End of the first contiguous in-radius visit.  This lets the last
        # discovered marker finish its hover before SEARCH is closed.
        visit_end_index = None
        if detected:
            visit_end_index = detection_index
            previous = detection_index
            for index in detected_indices[1:]:
                dt = samples[index].t - samples[previous].t
                if index == previous + 1 or dt <= hover_gap_tolerance:
                    visit_end_index = index
                    previous = index
                else:
                    break

        qualifying = []
        last_hover_index = (
            len(samples) - 1
            if max_hover_index is None
            else min(max_hover_index, len(samples) - 1)
        )
        for i, sample in enumerate(samples):
            if i > last_hover_index:
                break
            if (
                distances[i] <= hover_gate_radius
                and sample.speed <= hover_speed_threshold
            ):
                qualifying.append(i)

        episodes = []
        current = []
        for index in qualifying:
            if not current:
                current = [index]
                continue
            dt = samples[index].t - samples[current[-1]].t
            if dt <= hover_gap_tolerance:
                current.append(index)
            else:
                episodes.append(current)
                current = [index]
        if current:
            episodes.append(current)

        best = None
        for episode in episodes:
            start_index = episode[0]
            end_index = episode[-1]
            duration = samples[end_index].t - samples[start_index].t
            hover_rmse = rmse([distances[i] for i in episode])
            valid = (
                duration >= required_hover_time
                and hover_rmse is not None
                and hover_rmse <= hover_zero_score_rmse
            )
            candidate = (
                valid,
                duration,
                -(hover_rmse if hover_rmse is not None else 1e9),
                episode,
                hover_rmse,
            )
            if best is None or candidate[:3] > best[:3]:
                best = candidate

        if best is None:
            results[marker_id] = HoverResult(
                marker_id=marker_id,
                detected=detected,
                detection_time=detection_time,
                detection_index=detection_index,
                visit_end_index=visit_end_index,
                min_distance=minimum_distance,
            )
        else:
            valid, duration, _, episode, hover_rmse = best
            start_index = episode[0]
            end_index = episode[-1]
            results[marker_id] = HoverResult(
                marker_id=marker_id,
                detected=detected,
                detection_time=detection_time,
                detection_index=detection_index,
                visit_end_index=visit_end_index,
                valid=valid,
                start_index=start_index,
                end_index=end_index,
                start_time=samples[start_index].t,
                end_time=samples[end_index].t,
                duration=duration,
                rmse=hover_rmse,
                min_distance=minimum_distance,
            )

    return results


def find_arrival(samples, start_index, target, arrival_radius):
    for i in range(max(0, start_index), len(samples)):
        if horizontal_distance(samples[i], target) <= arrival_radius:
            return i
    return None


def infer_segments(samples, markers, home, search_end_index, arrival_radius):
    sequence = [(4, 3), (3, 2), (2, 1), (1, 0)]
    locations = {
        0: home,
        1: markers[1],
        2: markers[2],
        3: markers[3],
        4: markers[4],
    }
    results = []

    if search_end_index is None:
        for a, b in sequence:
            results.append(SegmentResult(
                name=f'{a}-{b}', source_id=a, destination_id=b,
                source_xy=locations[a], destination_xy=locations[b]
            ))
        return results

    marker4_index = find_arrival(
        samples, search_end_index, locations[4], arrival_radius
    )
    if marker4_index is None:
        for a, b in sequence:
            results.append(SegmentResult(
                name=f'{a}-{b}', source_id=a, destination_id=b,
                source_xy=locations[a], destination_xy=locations[b]
            ))
        return results

    cursor = marker4_index

    for sequence_index, (a, b) in enumerate(sequence):
        source = locations[a]
        destination = locations[b]
        start_index = cursor
        end_index = find_arrival(
            samples, start_index + 1, destination, arrival_radius
        )

        if end_index is None:
            errors = [
                point_segment_distance(
                    (samples[i].x, samples[i].y), source, destination
                )
                for i in range(start_index, len(samples))
            ]
            results.append(SegmentResult(
                name=f'{a}-{b}', source_id=a, destination_id=b,
                source_xy=source, destination_xy=destination,
                start_index=start_index, end_index=len(samples) - 1,
                start_time=samples[start_index].t,
                end_time=samples[-1].t,
                completed=False,
                rmse=rmse(errors),
                mean_error=mean_value(errors),
                max_error=max(errors) if errors else None,
                sample_count=len(errors),
            ))
            for aa, bb in sequence[sequence_index + 1:]:
                results.append(SegmentResult(
                    name=f'{aa}-{bb}', source_id=aa, destination_id=bb,
                    source_xy=locations[aa], destination_xy=locations[bb]
                ))
            break

        errors = [
            point_segment_distance(
                (samples[i].x, samples[i].y), source, destination
            )
            for i in range(start_index, end_index + 1)
        ]
        results.append(SegmentResult(
            name=f'{a}-{b}', source_id=a, destination_id=b,
            source_xy=source, destination_xy=destination,
            start_index=start_index, end_index=end_index,
            start_time=samples[start_index].t,
            end_time=samples[end_index].t,
            completed=True,
            rmse=rmse(errors),
            mean_error=mean_value(errors),
            max_error=max(errors) if errors else None,
            sample_count=len(errors),
        ))
        cursor = end_index

    return results


# ============================================================================
# REAL-TIME ROS 2 SCORER
# ============================================================================

class CompetitionScorer(Node):

    def __init__(self):
        super().__init__('competition_score')

        # ------------------------------------------------------------------
        # Topics / geometry
        # ------------------------------------------------------------------
        self.declare_parameter('pose_topic', '/drone/estimated_pose')
        self.declare_parameter('event_topic', '/competition/mission_event')
        self.declare_parameter('expected_frame', 'world')

        self.declare_parameter(
            'grid_x', [0.0, 3.0, 6.0, 9.0, 12.0, 15.0, 18.0, 21.0, 24.0]
        )
        self.declare_parameter('grid_y', [0.0, 3.0, 6.0, 9.0, 12.0, 15.0])
        # Fallback home is used only when auto_home is disabled.
        self.declare_parameter('home', [0.0, 0.0])
        self.declare_parameter('auto_home', True)
        self.declare_parameter('home_capture_duration', 1.0)

        # Inclusive competition-field boundary. A drone at (0, 15), (0, 0),
        # (24, 15), etc. is already inside and scoring starts immediately.
        self.declare_parameter('field_x_min', 0.0)
        self.declare_parameter('field_x_max', 24.0)
        self.declare_parameter('field_y_min', 0.0)
        self.declare_parameter('field_y_max', 15.0)

        self.declare_parameter('marker_1', [3.0, 12.0])
        self.declare_parameter('marker_2', [12.0, 6.0])
        self.declare_parameter('marker_3', [21.0, 3.0])
        self.declare_parameter('marker_4', [18.0, 9.0])

        # ------------------------------------------------------------------
        # Scoring parameters — matched to offline scorer defaults
        # ------------------------------------------------------------------
        self.declare_parameter('target_time', 180.0)
        self.declare_parameter('maximum_time', 600.0)
        self.declare_parameter('target_altitude', 2.0)

        self.declare_parameter('detection_radius', 1.0)
        self.declare_parameter('required_hover_time', 4.0)
        self.declare_parameter('hover_gate_radius', 1.0)
        self.declare_parameter('hover_speed_threshold', 0.25)
        self.declare_parameter('hover_gap_tolerance', 0.35)
        self.declare_parameter('hover_full_score_rmse', 0.20)
        self.declare_parameter('hover_zero_score_rmse', 0.50)

        self.declare_parameter('grid_error_limit', 0.75)
        self.declare_parameter('segment_error_limit', 0.75)
        self.declare_parameter('segment_arrival_radius', 1.0)

        self.declare_parameter('altitude_full_score_rmse', 0.20)
        self.declare_parameter('altitude_zero_score_rmse', 0.40)

        # Live status reporting only; final scoring always re-runs the same
        # trajectory inference used by the offline scorer.
        self.declare_parameter('live_status_period', 2.0)

        self.grid_x = list(self.get_parameter('grid_x').value)
        self.grid_y = list(self.get_parameter('grid_y').value)
        self.configured_home = tuple(float(v) for v in self.get_parameter('home').value)
        self.auto_home = bool(self.get_parameter('auto_home').value)
        self.home_capture_duration = max(0.0, float(self.get_parameter('home_capture_duration').value))
        self.field_x_min = float(self.get_parameter('field_x_min').value)
        self.field_x_max = float(self.get_parameter('field_x_max').value)
        self.field_y_min = float(self.get_parameter('field_y_min').value)
        self.field_y_max = float(self.get_parameter('field_y_max').value)
        self.home = self.configured_home
        self.markers = {
            i: tuple(float(v) for v in self.get_parameter(f'marker_{i}').value)
            for i in range(1, 5)
        }

        self.target_time = float(self.get_parameter('target_time').value)
        self.maximum_time = float(self.get_parameter('maximum_time').value)
        self.target_altitude = float(self.get_parameter('target_altitude').value)
        self.detection_radius = float(self.get_parameter('detection_radius').value)
        self.required_hover_time = float(self.get_parameter('required_hover_time').value)
        self.hover_gate_radius = float(self.get_parameter('hover_gate_radius').value)
        self.hover_speed_threshold = float(self.get_parameter('hover_speed_threshold').value)
        self.hover_gap_tolerance = float(self.get_parameter('hover_gap_tolerance').value)
        self.hover_full_score_rmse = float(self.get_parameter('hover_full_score_rmse').value)
        self.hover_zero_score_rmse = float(self.get_parameter('hover_zero_score_rmse').value)
        self.grid_error_limit = float(self.get_parameter('grid_error_limit').value)
        self.segment_error_limit = float(self.get_parameter('segment_error_limit').value)
        self.segment_arrival_radius = float(self.get_parameter('segment_arrival_radius').value)
        self.altitude_full_score_rmse = float(self.get_parameter('altitude_full_score_rmse').value)
        self.altitude_zero_score_rmse = float(self.get_parameter('altitude_zero_score_rmse').value)
        self.expected_frame = str(self.get_parameter('expected_frame').value)

        # Run information supplied by competition_gui.
        self.team = None
        self.team_id = None
        self.run_id = None
        self.run_output_dir = None
        self.run_configured = False

        self._frame_warned = False
        self.reset_run_data()

        # Run config must survive late subscriber joining.
        config_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Scoring trajectory is important; lidar_drone_tracker publishes this
        # topic RELIABLE, so subscribe RELIABLE rather than sensor-data QoS.
        pose_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        event_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            String, '/competition/run_config', self.run_config_callback, config_qos
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('pose_topic').value,
            self.pose_callback,
            pose_qos,
        )
        self.create_subscription(
            String,
            self.get_parameter('event_topic').value,
            self.event_callback,
            event_qos,
        )

        status_period = max(0.5, float(self.get_parameter('live_status_period').value))
        self.status_timer = self.create_timer(status_period, self.live_status_callback)

        self.get_logger().info('Competition scorer ready (automatic trajectory inference).')
        self.get_logger().info('Pose input: /drone/estimated_pose')
        self.get_logger().info(
            'Mission events retained for START/FINISH/ABORT and annotations; '
            'marker/hover/segment scoring is inferred from trajectory.'
        )
        self.get_logger().info('Waiting for /competition/run_config ...')

    # ========================================================================
    # Run management
    # ========================================================================

    def reset_run_data(self):
        self.started = False
        self.finished = False
        self.start_time = None
        self.samples: List[Sample] = []
        self.event_log = []

        # Live-only inference/status. Final score is recomputed from samples.
        self.live_detected = set()
        self.live_valid_hover = set()
        self.live_hover = {
            i: {
                'active': False,
                'start_t': None,
                'last_t': None,
                'errors': [],
                'best_duration': 0.0,
                'best_rmse': None,
            }
            for i in range(1, 5)
        }
        self.live_last_sample = None
        self.live_search_complete_announced = False
        self.live_hover_scoring_open = True
        self.live_final_visit_marker = None
        self.live_final_visit_last_t = None
        self.home_locked = False
        self.home_sample_count = 0

        # Field entry is independent of home. Pre-entry poses are retained only
        # to establish home; competition performance starts at first inclusive
        # field-boundary pose.
        self.field_entered = False
        self.field_entry_index = None
        self.field_entry_time = None

    def clock_time(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def elapsed(self):
        if self.start_time is None:
            return 0.0
        return self.clock_time() - self.start_time

    def run_config_callback(self, msg):
        try:
            config = json.loads(msg.data)
            team_name = str(config['team_name'])
            team_id = str(config['team_id'])
            run_id = int(config['run_id'])
            output_dir = Path(config['output_dir']).expanduser()
        except (KeyError, ValueError, TypeError, json.JSONDecodeError) as e:
            self.get_logger().error(f'Invalid run configuration: {e}')
            return

        if self.started:
            self.get_logger().warn(
                'Received new run configuration while scoring is active. Ignoring it.'
            )
            return

        try:
            output_dir.mkdir(parents=True, exist_ok=True)
        except Exception as e:
            self.get_logger().error(f'Cannot create output directory {output_dir}: {e}')
            return

        self.team = team_name
        self.team_id = team_id
        self.run_id = run_id
        self.run_output_dir = output_dir
        self.run_configured = True

        self.get_logger().info(
            f'Scorer configured: {self.team} ({self.team_id}), run {self.run_id}'
        )
        self.get_logger().info(f'Scorer output directory: {self.run_output_dir}')

    # ========================================================================
    # Pose input + live automatic inference
    # ========================================================================

    def _inside_competition_field(self, x, y):
        """True on the field interior OR boundary (inclusive bounds)."""
        return (
            self.field_x_min <= x <= self.field_x_max
            and self.field_y_min <= y <= self.field_y_max
        )

    def pose_callback(self, msg):
        if not self.started or self.finished:
            return

        frame_id = str(msg.header.frame_id).strip()
        if (
            self.expected_frame
            and frame_id
            and frame_id != self.expected_frame
            and not self._frame_warned
        ):
            self.get_logger().warn(
                f'Pose frame is "{frame_id}" but expected "{self.expected_frame}". '
                'Scoring continues; verify the coordinate frame.'
            )
            self._frame_warned = True

        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        z = float(msg.pose.pose.position.z)
        if not all(math.isfinite(v) for v in (x, y, z)):
            self.get_logger().warn('Ignoring non-finite drone pose.')
            return

        t = self.elapsed()
        if self.samples and t <= self.samples[-1].t:
            # ROS clock should be monotonic in normal operation; preserve a
            # strictly increasing trajectory for velocity estimation.
            t = self.samples[-1].t + 1e-6

        sample = Sample(index=len(self.samples), t=t, x=x, y=y, z=z)
        self.samples.append(sample)

        if (
            not self.field_entered
            and self._inside_competition_field(sample.x, sample.y)
        ):
            self.field_entered = True
            self.field_entry_index = sample.index
            self.field_entry_time = sample.t
            self.get_logger().info(
                f'[AUTO] Competition field entered at t={sample.t:.2f}s, '
                f'x={sample.x:.3f}m, y={sample.y:.3f}m. '
                'Performance scoring starts now.'
            )

        # Automatically establish this run's home from the initial LiDAR poses.
        # The home used by tracing is horizontal XY only; takeoff altitude is not
        # part of the final return-home segment score.
        if self.auto_home and not self.home_locked:
            if sample.t >= self.home_capture_duration:
                self.home, self.home_sample_count = infer_home_from_initial_samples(
                    self.samples, self.home_capture_duration
                )
                self.home_locked = True
                self.get_logger().info(
                    f'[AUTO] Home locked at x={self.home[0]:.3f}m, '
                    f'y={self.home[1]:.3f}m from {self.home_sample_count} initial poses.'
                )

        if self.field_entered:
            self._update_live_inference(sample)
            self.live_last_sample = sample

    def _update_live_inference(self, sample):
        """Immediate feedback only. Final scoring uses full-trajectory inference."""
        speed = 0.0
        if self.live_last_sample is not None:
            dt = sample.t - self.live_last_sample.t
            if dt > 1e-9:
                dx = sample.x - self.live_last_sample.x
                dy = sample.y - self.live_last_sample.y
                speed = math.sqrt(dx * dx + dy * dy) / dt
        sample.speed = speed

        # Once all four markers have been discovered, keep SEARCH hover credit
        # open only until the first visit to that last marker ends.  The final
        # visit is considered over after it has been outside the detection
        # radius for longer than the normal gap tolerance.
        if (
            self.live_hover_scoring_open
            and self.live_final_visit_marker is not None
            and self.live_final_visit_last_t is not None
            and sample.t - self.live_final_visit_last_t
            > self.hover_gap_tolerance
        ):
            final_marker = self.live_final_visit_marker
            final_hover = self.live_hover[final_marker]
            if final_hover['active'] and final_hover['last_t'] is not None:
                self._finalize_live_hover_episode(
                    final_marker, final_hover['last_t']
                )
            self.live_hover_scoring_open = False

        for marker_id, target in self.markers.items():
            d = horizontal_distance(sample, target)

            if d <= self.detection_radius and marker_id not in self.live_detected:
                self.live_detected.add(marker_id)
                self.get_logger().info(
                    f'[AUTO] Marker {marker_id} detected at t={sample.t:.2f}s, '
                    f'distance={d:.3f}m'
                )
                if len(self.live_detected) == 4:
                    self.live_final_visit_marker = marker_id

            if (
                marker_id == self.live_final_visit_marker
                and d <= self.detection_radius
            ):
                self.live_final_visit_last_t = sample.t

            h = self.live_hover[marker_id]
            qualifying = (
                self.live_hover_scoring_open
                and d <= self.hover_gate_radius
                and speed <= self.hover_speed_threshold
            )

            if qualifying:
                if not h['active']:
                    h['active'] = True
                    h['start_t'] = sample.t
                    h['errors'] = [d]
                else:
                    # If the last qualifying sample is too old, close the old
                    # episode first and start a new one.
                    if (
                        h['last_t'] is not None
                        and sample.t - h['last_t'] > self.hover_gap_tolerance
                    ):
                        self._finalize_live_hover_episode(marker_id, h['last_t'])
                        h['active'] = True
                        h['start_t'] = sample.t
                        h['errors'] = [d]
                    else:
                        h['errors'].append(d)
                h['last_t'] = sample.t
            elif h['active'] and h['last_t'] is not None:
                if sample.t - h['last_t'] > self.hover_gap_tolerance:
                    self._finalize_live_hover_episode(marker_id, h['last_t'])

        if len(self.live_valid_hover) == 4 and not self.live_search_complete_announced:
            self.live_search_complete_announced = True
            self.get_logger().info(
                '[AUTO] All four valid hover episodes found. '
                'Descending trace 4->3->2->1->home will be inferred at FINISH.'
            )

    def _finalize_live_hover_episode(self, marker_id, end_t):
        h = self.live_hover[marker_id]
        if not h['active'] or h['start_t'] is None:
            return

        duration = max(0.0, end_t - h['start_t'])
        hover_rmse = rmse(h['errors'])
        valid = (
            duration >= self.required_hover_time
            and hover_rmse is not None
            and hover_rmse <= self.hover_zero_score_rmse
        )

        if duration > h['best_duration']:
            h['best_duration'] = duration
            h['best_rmse'] = hover_rmse

        if valid and marker_id not in self.live_valid_hover:
            self.live_valid_hover.add(marker_id)
            self.get_logger().info(
                f'[AUTO] Marker {marker_id}: valid hover, '
                f'duration={duration:.2f}s, RMSE={hover_rmse:.3f}m'
            )

        h['active'] = False
        h['start_t'] = None
        h['last_t'] = None
        h['errors'] = []

    def live_status_callback(self):
        if not self.started or self.finished or not self.samples:
            return
        self.get_logger().info(
            f'[AUTO STATUS] t={self.samples[-1].t:.1f}s, '
            f'poses={len(self.samples)}, '
            f'detected={sorted(self.live_detected)}, '
            f'valid_hovers={sorted(self.live_valid_hover)}'
        )

    # ========================================================================
    # Mission events
    # ========================================================================

    def event_callback(self, msg):
        event = msg.data.strip()
        now = self.clock_time()
        self.get_logger().info(f'Mission event: {event}')

        if event == 'START':
            if not self.run_configured:
                self.get_logger().error(
                    'START received before /competition/run_config. Cannot begin scoring.'
                )
                return

            self.reset_run_data()
            self.home = self.configured_home
            self.started = True
            self.finished = False
            self.start_time = now
            self.event_log.append((0.0, event))
            self.get_logger().info(
                f'Scoring STARTED: {self.team} ({self.team_id}), run {self.run_id}'
            )
            return

        if not self.started or self.finished:
            return

        self.event_log.append((self.elapsed(), event))

        if event == 'FINISH':
            self.finish(aborted=False)
            return

        if event == 'ABORT':
            self.finish(aborted=True)
            return

        # The following events are intentionally retained as annotations.
        # They no longer control the final score, so a missing student event
        # cannot force a physically good trajectory to zero.
        if event in ('SEARCH_START', 'SEARCH_RESUME', 'RETURN_HOME'):
            return

        parts = event.split(':')
        if (
            len(parts) in (2, 3)
            and parts[0] in ('MARKER_DETECTED', 'HOVER_START', 'HOVER_END', 'TRACE_START')
        ):
            return

        self.get_logger().warn(f'Unrecognized mission event: {event}')

    # ========================================================================
    # Final automatic evaluation
    # ========================================================================

    def evaluate_trajectory(self, completion_time):
        if len(self.samples) < 2:
            raise RuntimeError(
                f'Only {len(self.samples)} valid pose samples were received. '
                'At least two are required for scoring.'
            )

        # Find the first pose inside or on the inclusive field boundary.
        entry_global_index = None
        for i, s in enumerate(self.samples):
            if self._inside_competition_field(s.x, s.y):
                entry_global_index = i
                break

        if entry_global_index is None:
            raise RuntimeError(
                'The drone never entered the competition field; '
                'no performance-scoring trajectory exists.'
            )

        entry_time = self.samples[entry_global_index].t

        # Performance samples begin exactly at field entry. Time is normalized
        # so t=0 is field entry. If the drone started on a boundary such as
        # (0,15), the first pose is inside and scoring starts immediately.
        samples = [
            Sample(
                index=j,
                t=s.t - entry_time,
                x=s.x,
                y=s.y,
                z=s.z,
            )
            for j, s in enumerate(self.samples[entry_global_index:])
        ]
        calculate_sample_speeds(samples)

        # First pass: marker DETECTION only determines SEARCH completion.
        preliminary_results = infer_marker_hovers(
            samples=samples,
            markers=self.markers,
            detection_radius=self.detection_radius,
            hover_gate_radius=self.hover_gate_radius,
            hover_speed_threshold=self.hover_speed_threshold,
            required_hover_time=self.required_hover_time,
            hover_zero_score_rmse=self.hover_zero_score_rmse,
            hover_gap_tolerance=self.hover_gap_tolerance,
            max_hover_index=None,
        )

        detected = {
            marker_id for marker_id, result in preliminary_results.items()
            if result.detected
        }

        # SEARCH closes at the end of the first visit to the last-discovered
        # marker. Closing it at that marker's first detection would leave only
        # one sample for its hover and make the fourth hover impossible.
        # Hover validity itself still does not control search completion.
        if len(detected) == 4:
            first_visit_end_indices = [
                preliminary_results[marker_id].visit_end_index
                for marker_id in (1, 2, 3, 4)
            ]
            search_end_index = max(first_visit_end_indices)
            search_end_time = samples[search_end_index].t
        else:
            search_end_index = None
            search_end_time = None

        # Second pass: HOVER is evaluated only during SEARCH. Any hover/stopping
        # over markers during 4->3->2->1->home is excluded permanently.
        hover_results = infer_marker_hovers(
            samples=samples,
            markers=self.markers,
            detection_radius=self.detection_radius,
            hover_gate_radius=self.hover_gate_radius,
            hover_speed_threshold=self.hover_speed_threshold,
            required_hover_time=self.required_hover_time,
            hover_zero_score_rmse=self.hover_zero_score_rmse,
            hover_gap_tolerance=self.hover_gap_tolerance,
            max_hover_index=search_end_index,
        )

        valid_hover = {
            marker_id for marker_id, result in hover_results.items()
            if result.valid
        }

        if self.auto_home:
            home, home_sample_count = infer_home_from_initial_samples(
                samples, self.home_capture_duration
            )
        else:
            home = self.configured_home
            home_sample_count = 0

        # Keep self.home synchronized for logging/status after final evaluation.
        self.home = home

        segments = infer_segments(
            samples=samples,
            markers=self.markers,
            home=home,
            search_end_index=search_end_index,
            arrival_radius=self.segment_arrival_radius,
        )

        # Hover samples are excluded from grid RMSE, just as offline scorer.
        hover_indices = set()
        for result in hover_results.values():
            if result.start_index is not None and result.end_index is not None:
                hover_indices.update(range(result.start_index, result.end_index + 1))

        search_last_index = (
            len(samples) - 1 if search_end_index is None else search_end_index
        )
        grid_error_by_index = {}
        grid_errors = []
        for i in range(search_last_index + 1):
            if i in hover_indices:
                continue
            e = nearest_grid_error(
                samples[i].x, samples[i].y, self.grid_x, self.grid_y
            )
            grid_errors.append(e)
            grid_error_by_index[i] = e
        grid_rmse = rmse(grid_errors)

        # To match the offline scorer, altitude is evaluated over all recorded
        # mission samples rather than relying on externally supplied states.
        altitude_errors = [abs(s.z - self.target_altitude) for s in samples]
        altitude_rmse = rmse(altitude_errors)

        all_segments_complete = (
            len(segments) == 4 and all(segment.completed for segment in segments)
        )
        segment_error_by_index = {}
        segment_errors = []
        if all_segments_complete:
            for segment in segments:
                for i in range(segment.start_index, segment.end_index + 1):
                    e = point_segment_distance(
                        (samples[i].x, samples[i].y),
                        segment.source_xy,
                        segment.destination_xy,
                    )
                    segment_errors.append(e)
                    segment_error_by_index[i] = e
            segment_rmse = rmse(segment_errors)
        else:
            segment_rmse = None

        hover_scores = {}
        for marker_id in (1, 2, 3, 4):
            result = hover_results[marker_id]
            hover_scores[marker_id] = (
                tolerance_score(
                    result.rmse,
                    self.hover_full_score_rmse,
                    self.hover_zero_score_rmse,
                )
                if result.valid else 0.0
            )
        hover_score = sum(hover_scores.values()) / 4.0

        # Mission-time scoring starts at field entry, not at START when the
        # drone may still be flying in from an outside takeoff location.
        scoring_completion_time = max(0.0, completion_time - entry_time)
        if scoring_completion_time <= self.target_time:
            time_score = 100.0
        elif scoring_completion_time >= self.maximum_time:
            time_score = 0.0
        else:
            time_score = (
                100.0 * (self.maximum_time - scoring_completion_time)
                / (self.maximum_time - self.target_time)
            )

        scores = {
            'grid': score(grid_rmse, self.grid_error_limit),
            'hover': hover_score,
            'segments': (
                score(segment_rmse, self.segment_error_limit)
                if all_segments_complete else 0.0
            ),
            'altitude': tolerance_score(
                altitude_rmse,
                self.altitude_full_score_rmse,
                self.altitude_zero_score_rmse,
            ),
            'time': time_score,
        }

        missing_markers = 4 - len(detected)
        invalid_hovers = len(detected - valid_hover)
        penalty = 10.0 * missing_markers + 5.0 * invalid_hovers

        final_score = (
            0.20 * scores['grid']
            + 0.25 * scores['hover']
            + 0.15 * scores['segments']
            + 0.25 * scores['altitude']
            + 0.15 * scores['time']
            - penalty
        )
        final_score = max(0.0, min(100.0, final_score))

        return {
            'samples': samples,
            'field_entry_global_index': entry_global_index,
            'field_entry_time_from_start_s': entry_time,
            'scoring_completion_time_s': scoring_completion_time,
            'home': home,
            'home_sample_count': home_sample_count,
            'hover_results': hover_results,
            'detected': detected,
            'valid_hover': valid_hover,
            'search_end_index': search_end_index,
            'search_end_time': search_end_time,
            'segments': segments,
            'all_segments_complete': all_segments_complete,
            'grid_rmse': grid_rmse,
            'grid_error_by_index': grid_error_by_index,
            'segment_rmse': segment_rmse,
            'segment_error_by_index': segment_error_by_index,
            'altitude_rmse': altitude_rmse,
            'hover_scores': hover_scores,
            'scores': scores,
            'missing_markers': missing_markers,
            'invalid_hovers': invalid_hovers,
            'penalty': penalty,
            'final_score': final_score,
        }

    def finish(self, aborted=False):
        if not self.started or self.finished:
            return

        # Close any live hover episode only for status/logging. Final inference
        # independently reconstructs all episodes from the stored trajectory.
        for marker_id, h in self.live_hover.items():
            if h['active'] and h['last_t'] is not None:
                self._finalize_live_hover_episode(marker_id, h['last_t'])

        completion_time = self.elapsed()
        self.finished = True

        try:
            evaluation = self.evaluate_trajectory(completion_time)
        except Exception as e:
            self.get_logger().error(f'Cannot evaluate mission: {e}')
            evaluation = None

        if self.run_output_dir is None:
            self.get_logger().error(
                'No run output directory was supplied by GUI. Result cannot be saved.'
            )
            self.started = False
            return

        folder = self.run_output_dir
        folder.mkdir(parents=True, exist_ok=True)

        if evaluation is None:
            result = {
                'team_name': self.team,
                'team_id': self.team_id,
                'run_id': self.run_id,
                'status': 'ABORTED' if aborted else 'ERROR',
                'completion_time_s': completion_time,
                'final_score': 0.0,
                'error': 'Insufficient or invalid trajectory data.',
            }
            result_file = folder / 'result.json'
            with result_file.open('w', encoding='utf-8') as f:
                json.dump(result, f, indent=2, allow_nan=False)
            self.started = False
            return

        samples = evaluation['samples']
        hover_results = evaluation['hover_results']
        segments = evaluation['segments']
        scores = evaluation['scores']
        final_score = 0.0 if aborted else evaluation['final_score']

        # ------------------------------------------------------------------
        # scoring_trajectory.csv
        # ------------------------------------------------------------------
        trajectory_file = folder / 'scoring_trajectory.csv'
        with trajectory_file.open('w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                'time_s', 'x_m', 'y_m', 'z_m', 'speed_mps',
                'grid_error_m', 'segment_error_m', 'altitude_error_m'
            ])
            for i, s in enumerate(samples):
                writer.writerow([
                    s.t, s.x, s.y, s.z, s.speed,
                    evaluation['grid_error_by_index'].get(i, ''),
                    evaluation['segment_error_by_index'].get(i, ''),
                    abs(s.z - self.target_altitude),
                ])

        # ------------------------------------------------------------------
        # event_log.csv — useful diagnostic/annotation, not score authority
        # ------------------------------------------------------------------
        event_file = folder / 'mission_events.csv'
        with event_file.open('w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['time_s', 'event'])
            writer.writerows(self.event_log)

        result = {
            'mode': 'realtime_automatic_inference',
            'team_name': self.team,
            'team_id': self.team_id,
            'run_id': self.run_id,
            'status': 'ABORTED' if aborted else 'FINISHED',
            'mission_complete': evaluation['all_segments_complete'],
            'completion_time_s': completion_time,
            'search_end_time_s': evaluation['search_end_time'],
            'pose_sample_count': len(samples),
            'home_xy_m': [evaluation['home'][0], evaluation['home'][1]],
            'home_source': ('initial_trajectory_average' if self.auto_home else 'configured_parameter'),
            'home_capture_duration_s': self.home_capture_duration if self.auto_home else None,
            'home_sample_count': evaluation['home_sample_count'],
            'detected_markers': sorted(evaluation['detected']),
            'valid_hover_markers': sorted(evaluation['valid_hover']),
            'grid_rmse_m': evaluation['grid_rmse'],
            'segment_rmse_m': evaluation['segment_rmse'],
            'altitude_rmse_m': evaluation['altitude_rmse'],
            'hover_rmse_m': {
                str(i): hover_results[i].rmse for i in range(1, 5)
            },
            'hover_duration_s': {
                str(i): hover_results[i].duration for i in range(1, 5)
            },
            'marker_min_distance_m': {
                str(i): hover_results[i].min_distance for i in range(1, 5)
            },
            'segment_rmse_individual_m': {
                segment.name: segment.rmse for segment in segments
            },
            'segment_completed': {
                segment.name: segment.completed for segment in segments
            },
            'scores': scores,
            'penalty': evaluation['penalty'],
            'penalty_detail': {
                'missing_markers': evaluation['missing_markers'],
                'invalid_hovers': evaluation['invalid_hovers'],
                'missing_marker_penalty_each': 10.0,
                'invalid_hover_penalty_each': 5.0,
            },
            'weights': {
                'grid': 0.20,
                'hover': 0.25,
                'segments': 0.15,
                'altitude': 0.25,
                'time': 0.15,
            },
            'thresholds': {
                'target_altitude_m': self.target_altitude,
                'auto_home': self.auto_home,
                'home_capture_duration_s': self.home_capture_duration,
                'target_time_s': self.target_time,
                'maximum_time_s': self.maximum_time,
                'required_hover_time_s': self.required_hover_time,
                'detection_radius_m': self.detection_radius,
                'hover_gate_radius_m': self.hover_gate_radius,
                'hover_speed_threshold_mps': self.hover_speed_threshold,
                'hover_gap_tolerance_s': self.hover_gap_tolerance,
                'hover_full_score_rmse_m': self.hover_full_score_rmse,
                'hover_zero_score_rmse_m': self.hover_zero_score_rmse,
                'grid_error_limit_m': self.grid_error_limit,
                'segment_error_limit_m': self.segment_error_limit,
                'segment_arrival_radius_m': self.segment_arrival_radius,
                'altitude_full_score_rmse_m': self.altitude_full_score_rmse,
                'altitude_zero_score_rmse_m': self.altitude_zero_score_rmse,
            },
            'final_score': final_score,
            'note': (
                'Final marker/hover/search/trace evaluation is inferred from '
                '/drone/estimated_pose using the same geometry/speed logic as '
                'offline_compet_score.py. Mission events are retained for '
                'START/FINISH/ABORT and diagnostics. Home XY is automatically '
                'estimated from the initial trajectory when auto_home is enabled.'
            ),
        }

        result_file = folder / 'result.json'
        with result_file.open('w', encoding='utf-8') as f:
            json.dump(result, f, indent=2, allow_nan=False)

        # Compact summary CSV for easy inspection.
        summary_file = folder / 'score_summary.csv'
        with summary_file.open('w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['metric', 'value'])
            writer.writerow(['mission_complete', evaluation['all_segments_complete']])
            writer.writerow(['home_x_m', evaluation['home'][0]])
            writer.writerow(['home_y_m', evaluation['home'][1]])
            writer.writerow(['home_sample_count', evaluation['home_sample_count']])
            writer.writerow(['completion_time_s', completion_time])
            writer.writerow(['detected_markers', sorted(evaluation['detected'])])
            writer.writerow(['valid_hover_markers', sorted(evaluation['valid_hover'])])
            writer.writerow(['grid_rmse_m', evaluation['grid_rmse']])
            writer.writerow(['segment_rmse_m', evaluation['segment_rmse']])
            writer.writerow(['altitude_rmse_m', evaluation['altitude_rmse']])
            writer.writerow(['grid_score', scores['grid']])
            writer.writerow(['hover_score', scores['hover']])
            writer.writerow(['segment_score', scores['segments']])
            writer.writerow(['altitude_score', scores['altitude']])
            writer.writerow(['time_score', scores['time']])
            writer.writerow(['penalty', evaluation['penalty']])
            writer.writerow(['final_score', final_score])

        self.get_logger().info('========================================')
        self.get_logger().info(f'Team: {self.team} ({self.team_id})')
        self.get_logger().info(f'Run: {self.run_id}')
        self.get_logger().info(f'Completion time: {completion_time:.2f} s')
        self.get_logger().info(
            f'Home used for return: x={evaluation["home"][0]:.3f}m, '
            f'y={evaluation["home"][1]:.3f}m '
            f'({"auto" if self.auto_home else "configured"})'
        )
        self.get_logger().info(
            f'Detected markers: {sorted(evaluation["detected"])}'
        )
        self.get_logger().info(
            f'Valid hovers: {sorted(evaluation["valid_hover"])}'
        )
        self.get_logger().info(
            f'Grid RMSE: {evaluation["grid_rmse"]}'
        )
        self.get_logger().info(
            f'Segment RMSE: {evaluation["segment_rmse"]}'
        )
        self.get_logger().info(
            f'Altitude RMSE: {evaluation["altitude_rmse"]}'
        )
        self.get_logger().info(
            'Component scores: '
            f'grid={scores["grid"]:.2f}, '
            f'hover={scores["hover"]:.2f}, '
            f'segments={scores["segments"]:.2f}, '
            f'altitude={scores["altitude"]:.2f}, '
            f'time={scores["time"]:.2f}'
        )
        self.get_logger().info(f'Penalty: {evaluation["penalty"]:.2f}')
        self.get_logger().info(f'Final score: {final_score:.2f} / 100')
        self.get_logger().info(f'Result saved: {result_file}')
        self.get_logger().info('========================================')

        self.started = False


# ============================================================================
# Main
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = CompetitionScorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
