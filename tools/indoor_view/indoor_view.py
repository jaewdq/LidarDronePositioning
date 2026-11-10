"""실내 케이지 트래커 확인용 표시 (RViz: indoor_view.rviz).

    python3 tools/indoor_view/indoor_view.py src/drone_localization/params/lidar_localization_os0_indoor.yaml &
    rviz2 -d tools/indoor_view/indoor_view.rviz

- 트래커 YAML 의 lidarN_* 로 map -> lidarN_sensor 정적 TF (드라이버의 sensor->lidar 와 합성)
- /viz/field (3x2 바닥 테두리), /viz/roi (트래커 ROI 상자), /viz/track (추정 궤적, 최근 30초)
- /viz/clear (std_msgs/Empty) 를 받으면 궤적과 위치 글자를 지운다 (UAV Tracking 의 START RUN 이 보낸다)
- /viz/positions 물체 추정 위치(SU)의 (x, y, z) 글자와 점, 추정이 나올 때마다 갱신"""
import sys, math, yaml, numpy as np, rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import TransformStamped, Point, PoseStamped, PoseWithCovarianceStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from std_msgs.msg import Empty

P = yaml.safe_load(open(sys.argv[1]))['lidar_drone_tracker']['ros__parameters']
SENSOR_TO_LIDAR = (np.array([0.0, 0.0, 0.038]), Rotation.from_euler('z', math.pi))   # 드라이버 static TF

def pt(x, y, z): return Point(x=float(x), y=float(y), z=float(z))

def text(ns, i, x, y, z, s, rgb, size=0.08):
    m = Marker(); m.header.frame_id = 'map'; m.ns = ns; m.id = i; m.type = Marker.TEXT_VIEW_FACING
    m.pose.position = pt(x, y, z); m.scale.z = size; m.text = s
    m.color.r, m.color.g, m.color.b = rgb; m.color.a = 1.0
    return m

def dot(ns, i, x, y, z, rgb, size=0.05):
    m = Marker(); m.header.frame_id = 'map'; m.ns = ns; m.id = i; m.type = Marker.SPHERE
    m.pose.position = pt(x, y, z); m.pose.orientation.w = 1.0; m.scale.x = m.scale.y = m.scale.z = size
    m.color.r, m.color.g, m.color.b = rgb; m.color.a = 1.0
    return m


class View(Node):
    def __init__(self):
        super().__init__('indoor_view')
        self.tf = StaticTransformBroadcaster(self)
        msgs = []
        for i in (1, 2):
            t = np.array([P[f'lidar{i}_{k}'] for k in 'xyz'])
            r = Rotation.from_euler('xyz', [P[f'lidar{i}_roll'], P[f'lidar{i}_pitch'], P[f'lidar{i}_yaw']])
            # map->sensor = map->lidar * (sensor->lidar)^-1
            rs = r * SENSOR_TO_LIDAR[1].inv()
            ts = t - rs.apply(SENSOR_TO_LIDAR[0])
            m = TransformStamped(); m.header.frame_id = 'map'; m.child_frame_id = f'lidar{i}_sensor'
            m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = map(float, ts)
            q = rs.as_quat(); m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = map(float, q)
            msgs.append(m)
        self.tf.sendTransform(msgs)
        self.pub_f = self.create_publisher(Marker, '/viz/field', 10)
        self.pub_r = self.create_publisher(Marker, '/viz/roi', 10)
        self.pub_p = self.create_publisher(Path, '/viz/track', 10)
        self.path = Path(); self.path.header.frame_id = 'map'
        self.create_subscription(PoseWithCovarianceStamped, '/drone/estimated_pose', self.on_pose, 10)
        self.pub_x = self.create_publisher(MarkerArray, '/viz/positions', 10)
        self.create_subscription(Empty, '/viz/clear', self.clear, 10)
        self.create_timer(1.0, self.markers)

    def clear(self, _msg):
        self.path.poses = []; self.path.header.stamp = self.get_clock().now().to_msg()
        self.pub_p.publish(self.path)
        gone = Marker(); gone.action = Marker.DELETEALL
        self.pub_x.publish(MarkerArray(markers=[gone]))

    def on_pose(self, m):
        p = PoseStamped(); p.header = m.header; p.pose = m.pose.pose
        # 개수가 아니라 시각으로 자른다 — 검출이 드문드문하면 300개가 몇 분치가 된다
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.path.poses = [q for q in self.path.poses
                           if 0 <= t - (q.header.stamp.sec + q.header.stamp.nanosec * 1e-9) <= 30.0] + [p]
        self.path.header.stamp = m.header.stamp
        self.pub_p.publish(self.path)
        q = m.pose.pose.position
        self.pub_x.publish(MarkerArray(markers=[
            dot('pos_dot', 0, q.x, q.y, q.z, (0.2, 1.0, 0.5), 0.06),
            text('pos_text', 0, q.x, q.y, q.z + 0.35, f'({q.x:.3f},{q.y:.3f},{q.z:.2f})', (1.0, 1.0, 1.0),
                 0.07)]))

    def markers(self):
        f = Marker(); f.header.frame_id = 'map'; f.ns = 'field'; f.type = Marker.LINE_STRIP
        f.scale.x = 0.03; f.color.g = 1.0; f.color.r = 0.3; f.color.a = 1.0
        f.points = [pt(x, y, 0) for x, y in [(0, 0), (3, 0), (3, 2), (0, 2), (0, 0)]]
        self.pub_f.publish(f)
        b = Marker(); b.header.frame_id = 'map'; b.ns = 'roi'; b.type = Marker.LINE_LIST
        b.scale.x = 0.01; b.color.r = 1.0; b.color.g = 0.8; b.color.a = 0.8
        X = (P['roi_x_min'], P['roi_x_max']); Y = (P['roi_y_min'], P['roi_y_max']); Z = (P['roi_z_min'], P['roi_z_max'])
        c = [(x, y, z) for x in X for y in Y for z in Z]
        for i in range(8):
            for j in range(i + 1, 8):
                if sum(a != bb for a, bb in zip(c[i], c[j])) == 1:
                    b.points += [pt(*c[i]), pt(*c[j])]
        self.pub_r.publish(b)

rclpy.init(); rclpy.spin(View())
