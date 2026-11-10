#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import time

def get_distance_meters(lat1, lon1, lat2, lon2):
    R = 6371000
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

class LogicNode(Node):
    def __init__(self):
        super().__init__('logic_node')

        # === PARAMETER CONFIG LINTASAN ===
        self.declare_parameter('lintasan_mode', 0) # 0 = A, 1 = B
        self.lintasan_mode = self.get_parameter('lintasan_mode').value

        if self.lintasan_mode == 0:
            self.get_logger().info('LOGIC NODE: Konfigurasi LINTASAN A Aktif')
            self.GATE_WAYPOINTS = [
                (-7.97865213, 112.73903117),  # WP 2 Lintasan A
                (-7.97858703, 112.73905496),  # WP 3 Lintasan A
                (-7.97855852, 112.73902933),  # WP 4 Lintasan A
                (-7.97852679, 112.73896902),  # WP 5 Lintasan A (Mulai Terbalik)
                (-7.97855616, 112.73894795),  # WP 6 Lintasan A
                (-7.97862501, 112.73894213),  # WP 7 Lintasan A
                (-7.97869388, 112.73901197),  # WP 1 Lintasan A
            ]
        else:
            self.get_logger().info('LOGIC NODE: Konfigurasi LINTASAN B Aktif')
            self.GATE_WAYPOINTS = [
                (-7.97810000, 112.73910000),  # WP 2 Lintasan B
                (-7.97820000, 112.73920000),  # WP 3 Lintasan B
                (-7.97830000, 112.73930000),  # WP 4 Lintasan B
                (-7.97840000, 112.73940000),  # WP 5 Lintasan B (Mulai Terbalik)
                (-7.97850000, 112.73950000),  # WP 6 Lintasan B
            ]

        # Mode terbalik aktif mulai dari indeks waypoint ke-4 (setelah melewati WP 4)
        self.REVERSED_MODE_START_INDEX = 3  
        self.BOX_SEARCH_AREA = (-7.94990, 112.61290) 
        
        self.SPEED_NORMAL = 1.0; self.SPEED_SEARCH = 0.8
        self.STOP_BOX_AREA = 20000
        
        self.gate_idx = 0
        self.last_gate_error = 9999.0; self.last_gate_time = 0
        self.last_box_msg = None; self.last_box_time = 0
        self.current_gps = None; self.current_heading = None

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub_cmd = self.create_publisher(Twist, '/rover_command', 10)
        
        # MENYAMAKAN NAMA TOPIK DENGAN YANG DIDENGAR OLEH VISION_NODE
        self.pub_gate_mode = self.create_publisher(Int32, '/change_lintasan_mode', 10)
        
        self.sub_gate = self.create_subscription(Float64, '/gate_error', self.gate_cb, 10)
        self.sub_box = self.create_subscription(Point, '/box_position', self.box_cb, 10)
        self.sub_gps = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_cb, sensor_qos)
        self.sub_head = self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.head_cb, sensor_qos)
        
        self.timer = self.create_timer(0.05, self.control_loop)

    def gate_cb(self, msg): self.last_gate_error = msg.data; self.last_gate_time = time.time()
    def box_cb(self, msg): self.last_box_msg = msg; self.last_box_time = time.time()
    def gps_cb(self, msg): self.current_gps = (msg.latitude, msg.longitude)
    def head_cb(self, msg): self.current_heading = msg.data

    # ================= FUZZY LOGIC STEERING ENGINE =================
    def fuzzy_membership_error(self, error, max_val):
        norm_err = max(min(error / float(max_val), 1.0), -1.0)
        if norm_err <= -0.5: ln = 1.0
        elif -0.5 < norm_err < 0.0: ln = -norm_err / 0.5
        else: ln = 0.0

        if -0.5 < norm_err <= 0.0: z = (norm_err + 0.5) / 0.5
        elif 0.0 < norm_err < 0.5: z = (0.5 - norm_err) / 0.5
        else: z = 0.0

        if norm_err >= 0.5: kp = 1.0
        elif 0.0 < norm_err < 0.5: kp = norm_err / 0.5
        else: kp = 0.0
        return ln, z, kp

    def get_fuzzy_steering(self, error, max_error_val, inverted_output=False):
        ln, z, kp = self.fuzzy_membership_error(error, max_error_val)
        STEER_SHARP_LEFT = 0.8; STEER_STRAIGHT = 0.0; STEER_SHARP_RIGHT = -0.8

        if not inverted_output:
            w1, out1 = ln, STEER_SHARP_LEFT
            w2, out2 = z, STEER_STRAIGHT
            w3, out3 = kp, STEER_SHARP_RIGHT
        else:
            w1, out1 = ln, STEER_SHARP_RIGHT
            w2, out2 = z, STEER_STRAIGHT
            w3, out3 = kp, STEER_SHARP_LEFT

        total_weight = w1 + w2 + w3
        if total_weight == 0: return 0.0
        return float((w1 * out1 + w2 * out2 + w3 * out3) / total_weight)
    # ===============================================================

    def control_loop(self):
        cmd = Twist()
        curr_time = time.time()
        if not self.current_gps or self.current_heading is None: return 

        # === LOGIKA ADAPTIF UNTUK PERTUKARAN WARNA BOLA ===
        # Mengecek apakah robot sudah melewati batas Waypoint pembalikan arah
        is_reversed = self.gate_idx >= self.REVERSED_MODE_START_INDEX

        if self.lintasan_mode == 0: # Jika Mode Awal Lintasan A
            # Sesudah WP 4 warna bola tertukar, maka gunakan pembacaan ala Lintasan B (mode 1)
            mode_kirim = 1 if is_reversed else 0
        else:                      # Jika Mode Awal Lintasan B
            # Sesudah WP 4 warna bola tertukar, maka gunakan pembacaan ala Lintasan A (mode 0)
            mode_kirim = 0 if is_reversed else 1

        mode_msg = Int32()
        mode_msg.data = mode_kirim
        self.pub_gate_mode.publish(mode_msg)
        # =================================================

        # Prioritas 1: Kotak / Box Docking
        box_active = (curr_time - self.last_box_time < 0.5)
        if box_active and self.last_box_msg.z == 2.0: 
            if self.last_box_msg.y >= self.STOP_BOX_AREA:
                cmd.linear.x = 0.0; cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd)
            else:
                turn = self.get_fuzzy_steering(self.last_box_msg.x, max_error_val=320, inverted_output=False)
                cmd.linear.x = 0.6; cmd.angular.z = turn
                self.pub_cmd.publish(cmd)
            return

        # Prioritas 2: Gerbang / Vision Gate
        ball_visible = (self.last_gate_error != 9999.0) and (curr_time - self.last_gate_time < 0.2)
        if ball_visible:
            turn = self.get_fuzzy_steering(-self.last_gate_error, max_error_val=320, inverted_output=False)
            speed = max(self.SPEED_NORMAL - (abs(turn) * 0.6), 0.3)
            cmd.linear.x = float(speed); cmd.angular.z = turn
            self.pub_cmd.publish(cmd)
        else:
            if self.gate_idx >= len(self.GATE_WAYPOINTS):
                self.navigate_gps_fuzzy(cmd, self.BOX_SEARCH_AREA, speed=0.6)
                return

            target = self.GATE_WAYPOINTS[self.gate_idx]
            dist = get_distance_meters(self.current_gps[0], self.current_gps[1], target[0], target[1])
            
            if dist < 3.0: 
                self.get_logger().info(f"✅ WP {self.gate_idx+1} Sukses Lewat. Berlanjut ke WP berikutnya.")
                self.gate_idx += 1
                return 

            self.navigate_gps_fuzzy(cmd, target, speed=self.SPEED_SEARCH)

    def navigate_gps_fuzzy(self, cmd, target, speed=1.0):
        bearing = get_bearing(self.current_gps[0], self.current_gps[1], target[0], target[1])
        heading_err = bearing - self.current_heading
        if heading_err > 180: heading_err -= 360
        if heading_err < -180: heading_err += 360
        
        turn = self.get_fuzzy_steering(heading_err, max_error_val=180.0, inverted_output=True)
        cmd.linear.x = speed; cmd.angular.z = turn
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args); node = LogicNode(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()