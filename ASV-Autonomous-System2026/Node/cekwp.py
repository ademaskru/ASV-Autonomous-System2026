#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math

# === FUNGSI MATEMATIKA GPS (Haversine & Bearing) ===
def get_distance_meters(lat1, lon1, lat2, lon2):
    R = 6371000 # Radius Bumi (meter)
    phi1 = math.radians(lat1); phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1); delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def get_bearing(lat1, lon1, lat2, lon2):
    lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])
    dLon = lon2 - lon1
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
    return (math.degrees(math.atan2(y, x)) + 360) % 360

class SimpleWaypointNode(Node):
    def __init__(self):
        super().__init__('simple_waypoint_node')
        self.get_logger().info('🚀 SIMPLE WAYPOINT MISSION STARTED (SINKRON DENGAN LOGIC_NODE)')
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # === 1. DAFTAR WAYPOINT (DISAMAKAN DENGAN ROUTE UTAMA LINTASAN A) ===
        # Jika Anda ingin menguji Lintasan B, silakan ganti koordinat ini dengan koordinat Lintasan B
        self.WAYPOINTS = [
            (-7.97865213, 112.73903117),  # WP 2 Lintasan A
            (-7.97858703, 112.73905496),  # WP 3 Lintasan A
            (-7.97855852, 112.73902933),  # WP 4 Lintasan A
            (-7.97852679, 112.73896902),  # WP 5 Lintasan A
            (-7.97855616, 112.73894795),  # WP 6 Lintasan A
            (-7.97862501, 112.73894213),  # WP 7 Lintasan A
            (-7.97869388, 112.73901197),  # WP 1 Lintasan A
        ]
        
        # === 2. CONFIG ===
        self.WP_RADIUS = 2.0      # Target radius (meter)
        self.SPEED = 0.8          # Kecepatan konstan
        self.Kp_heading = 1.0     # Gain P-Controller Steering
        
        self.wp_index = 0
        self.current_gps = None
        self.current_heading = None

        self.pub_cmd = self.create_publisher(Twist, '/rover_command', 10)
        self.sub_gps = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_cb, sensor_qos)
        self.sub_head = self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.head_cb, sensor_qos)
        
        self.timer = self.create_timer(0.1, self.control_loop) # 10Hz

    def gps_cb(self, msg): self.current_gps = (msg.latitude, msg.longitude)
    def head_cb(self, msg): self.current_heading = msg.data

    def control_loop(self):
        cmd = Twist()
        
        if not self.current_gps or self.current_heading is None:
            self.get_logger().warn("Menunggu GPS & Compass...", throttle_duration_sec=2)
            return

        if self.wp_index >= len(self.WAYPOINTS):
            self.get_logger().info("🏁 MISI PENGUJIAN SELESAI. DIAM DI TEMPAT.", throttle_duration_sec=5)
            cmd.linear.x = 0.0; cmd.angular.z = 0.0
            self.pub_cmd.publish(cmd)
            return

        target = self.WAYPOINTS[self.wp_index]
        dist = get_distance_meters(self.current_gps[0], self.current_gps[1], target[0], target[1])

        if dist < self.WP_RADIUS:
            self.get_logger().info(f"✅ SAMPAI DI WP {self.wp_index+1}! Lanjut ke WP berikutnya.")
            self.wp_index += 1
            return

        target_bearing = get_bearing(self.current_gps[0], self.current_gps[1], target[0], target[1])
        heading_error = target_bearing - self.current_heading
        
        if heading_error > 180: heading_error -= 360
        if heading_error < -180: heading_error += 360
        
        turn_output = math.radians(heading_error) * self.Kp_heading
        turn_output = max(min(turn_output, 0.8), -0.8)

        cmd.linear.x = self.SPEED
        cmd.angular.z = float(turn_output)
        self.pub_cmd.publish(cmd)
        
        self.get_logger().info(
            f"Go to WP {self.wp_index+1} | Dist: {dist:.1f}m | Err: {heading_error:.1f}deg", 
            throttle_duration_sec=0.5
        )

def main(args=None):
    rclpy.init(args=args); node = SimpleWaypointNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.pub_cmd.publish(Twist()) # Stop motor
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()