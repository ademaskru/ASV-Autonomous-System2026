#!/usr/bin/env python3
import math
import time
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, Int32

def get_distance_meters(lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = (
        math.sin(delta_phi / 2) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def get_bearing(lat1, lon1, lat2, lon2):
    lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])
    dLon = lon2 - lon1
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(
        lat2
    ) * math.cos(dLon)
    return (math.degrees(math.atan2(y, x)) + 360) % 360

class LogicNode(Node):
    def __init__(self):
        super().__init__('logic_node')

        # Mode Parameter: 0 = Lintasan A, 1 = Lintasan B
        self.declare_parameter('lintasan_mode', 1)
        self.lintasan_mode = self.get_parameter('lintasan_mode').value

        # ================= SETUP BLUEPRINT (CETAKAN WAYPOINT) =================
        if self.lintasan_mode == 0:
            self.get_logger().info('🚀 LOGIC NODE 5-MF: LINTASAN A AKTIF (Mode 0)')
            self.BLUEPRINT_HOME = (1.49217244, 102.12880176)
            self.BLUEPRINT_WPS = [
                    (1.49216673, 102.12885927),  # WP 2
                    (1.49208641, 102.12896747),  # WP 3
                    (1.49195760, 102.12891959),  # WP 4
                    (1.49200510, 102.12878840),  # WP 5
                    (1.49217410, 102.12879294),  # WP 6
            ]
        else:
            self.get_logger().info('🚀 LOGIC NODE 5-MF: LINTASAN B AKTIF (Mode 1)')
            self.BLUEPRINT_HOME = (1.49217244, 102.12880176)
            self.BLUEPRINT_WPS = [
                (1.49216673, 102.12885927),  # WP 2
                    (1.49208641, 102.12896747),  # WP 3
                    (1.49195760, 102.12891959),  # WP 4
                    (1.49200510, 102.12878840),  # WP 5
                    (1.49217410, 102.12879294),  # WP 6
            ]

        self.GATE_WAYPOINTS = []
        self.actual_home = None
        self.initial_heading = None  # Menyimpan arah hadap kapal awal
        self.waypoints_generated = False
        self.gps_init_count = 0  # Buffer tunggu GPS stabil

        self.REVERSED_MODE_START_INDEX = 9
        self.BOX_SEARCH_AREA = (-7.95300192, 112.61417154)

        self.SPEED_NORMAL = 1.0
        self.SPEED_SEARCH = 0.8
        self.STOP_BOX_AREA = 20000

        self.gate_idx = 0
        self.last_gate_error = 9999.0
        self.last_gate_time = 0
        self.last_box_msg = None
        self.last_box_time = 0
        self.current_gps = None
        self.current_heading = None

        # === LOGIKA DOCKING ===
        self.docking_state = 0 # 0: Cari, 1: Mendekat, 2: Menyamping (Strafing), 3: Selesai
        self.pivot_start_time = 0.0
        self.DOCKING_SIDE = "LEFT" # Ubah ke "RIGHT" jika lambung KANAN yang harus menyentuh bola

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub_cmd = self.create_publisher(Twist, '/rover_command', 10)
        self.pub_gate_mode = self.create_publisher(
            Int32, '/change_lintasan_mode', 10
        )
        self.sub_gate = self.create_subscription(
            Float64, '/gate_error', self.gate_cb, 10
        )
        self.sub_box = self.create_subscription(
            Point, '/box_position', self.box_cb, 10
        )
        self.sub_gps = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_cb,
            sensor_qos,
        )
        self.sub_head = self.create_subscription(
            Float64,
            '/mavros/global_position/compass_hdg',
            self.head_cb,
            sensor_qos,
        )
        self.timer = self.create_timer(0.05, self.control_loop)

    def generate_waypoints_from_blueprint(self):
        # Langsung gunakan koordinat asli tanpa digeser atau diputar
        self.GATE_WAYPOINTS = self.BLUEPRINT_WPS.copy()
        self.waypoints_generated = True
        
        self.get_logger().info("✅ MENGGUNAKAN WAYPOINT ORIGINAL (Tanpa Rotasi/Geser)!")
        
    def gate_cb(self, msg):
        self.last_gate_error = msg.data
        self.last_gate_time = time.time()

    def box_cb(self, msg):
        self.last_box_msg = msg
        self.last_box_time = time.time()

    def gps_cb(self, msg):
        if self.current_gps is None and msg.latitude != 0.0:
            self.get_logger().info(
                f"📡 [PIXHAWK INTERFACE] GPS Terhubung! Lat: {msg.latitude:.7f}, Lon: {msg.longitude:.7f}"
            )
        self.current_gps = (msg.latitude, msg.longitude)

        if not self.waypoints_generated and msg.latitude != 0.0 and self.current_heading is not None:
            self.gps_init_count += 1
            if self.gps_init_count > 10:    # diperbesar agar lebih stabil 
                self.actual_home = self.current_gps
                self.initial_heading = self.current_heading
                self.generate_waypoints_from_blueprint()

    def head_cb(self, msg):
        if self.current_heading is None:
            self.get_logger().info(
                f"🧭 [PIXHAWK INTERFACE] Kompas Terhubung! Heading: {msg.data:.2f}°"
            )
        self.current_heading = msg.data

    def fuzzy_membership_error_5mf(self, error, max_val):
        norm_err = max(min(error / float(max_val), 1.0), -1.0)

        if norm_err <= -0.6: ll = 1.0
        elif -0.6 < norm_err < -0.2: ll = (-0.2 - norm_err) / 0.4
        else: ll = 0.0

        if -0.6 < norm_err <= -0.2: ls = (norm_err + 0.6) / 0.4
        elif -0.2 < norm_err < 0.0: ls = (0.0 - norm_err) / 0.2
        else: ls = 0.0

        if -0.2 < norm_err <= 0.0: z = (norm_err + 0.2) / 0.2
        elif 0.0 < norm_err < 0.2: z = (0.2 - norm_err) / 0.2
        else: z = 0.0

        if 0.0 < norm_err <= 0.2: rs = norm_err / 0.2
        elif 0.2 < norm_err < 0.6: rs = (0.6 - norm_err) / 0.4
        else: rs = 0.0

        if norm_err >= 0.6: rl = 1.0
        elif 0.2 < norm_err < 0.6: rl = (norm_err - 0.2) / 0.4
        else: rl = 0.0

        return ll, ls, z, rs, rl

    def get_fuzzy_steering(self, error, max_error_val, inverted_output=False):
        ll, ls, z, rs, rl = self.fuzzy_membership_error_5mf(error, max_error_val)

        STEER_SHARP_LEFT = 0.8
        STEER_SOFT_LEFT = 0.35
        STEER_STRAIGHT = 0.0
        STEER_SOFT_RIGHT = -0.35
        STEER_SHARP_RIGHT = -0.8

        if not inverted_output:
            w1, out1 = ll, STEER_SHARP_LEFT
            w2, out2 = ls, STEER_SOFT_LEFT
            w3, out3 = z, STEER_STRAIGHT
            w4, out4 = rs, STEER_SOFT_RIGHT
            w5, out5 = rl, STEER_SHARP_RIGHT
        else:
            w1, out1 = ll, STEER_SHARP_RIGHT
            w2, out2 = ls, STEER_SOFT_RIGHT
            w3, out3 = z, STEER_STRAIGHT
            w4, out4 = rs, STEER_SOFT_LEFT
            w5, out5 = rl, STEER_SHARP_LEFT

        total_weight = w1 + w2 + w3 + w4 + w5
        if total_weight == 0:
            return 0.0
        return float((w1 * out1 + w2 * out2 + w3 * out3 + w4 * out4 + w5 * out5) / total_weight)

    def control_loop(self):
        cmd = Twist()
        # Default: Set Y ke 0 agar kapal tidak menggeser (strafing) tanpa sengaja
        cmd.linear.y = 0.0 
        curr_time = time.time()

        # --- PENGECEKAN KONEKSI DATA PIXHAWK ---
        if not self.current_gps or self.current_heading is None or not self.waypoints_generated:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            self.pub_cmd.publish(cmd)
            
            if not self.current_gps:
                self.get_logger().warn("⚠️ [MENUNGGU] Data GPS belum masuk!", throttle_duration_sec=2.0)
            elif self.current_heading is None:
                self.get_logger().warn("⚠️ [MENUNGGU] Data Kompas belum masuk!", throttle_duration_sec=2.0)
            else:
                self.get_logger().warn("⚠️ [MENUNGGU] Stabilisasi Home WP...", throttle_duration_sec=2.0)
            return

        # Sinkronisasi status mode ke vision_node
        mode_msg = Int32()
        mode_msg.data = self.lintasan_mode  
        self.pub_gate_mode.publish(mode_msg)

        # === PRIORITAS 1: BOX DOCKING (MANUVER MENYAMPING OMNI-X) ===
        box_active = (curr_time - self.last_box_time < 0.5)

        # STATE 2 & 3: Eksekusi Menyamping (Strafing)
        if self.docking_state == 2:
            if curr_time - self.pivot_start_time < 4.0:  # Durasi manuver (Bisa dinaikkan jika kurang lama nempelnya)
                # ---> REVISI 3: DORONGAN MAJU SAAT MENYAMPING <---
                cmd.linear.x = 0.15  # Kapal maju pelan sambil menggeser ke kanan
                cmd.angular.z = 0.0  
                
                if self.DOCKING_SIDE == "LEFT":
                    cmd.linear.y = 0.5   
                else:
                    cmd.linear.y = -0.5  # Menggeser kapal ke kanan (menuju bola)
                
                self.pub_cmd.publish(cmd)
                return
            else:
                self.docking_state = 3
                self.get_logger().info("✅ DOCKING SELESAI! Kapal telah bersandar di 3 bola.")
                
        if self.docking_state == 3:
            cmd.linear.x = 0.0; cmd.linear.y = 0.0; cmd.angular.z = 0.0
            self.pub_cmd.publish(cmd)
            return

        # STATE 1: Kamera melihat bola biru dan mulai mendekat lurus
        if box_active and self.last_box_msg and self.last_box_msg.z == 2.0:
            if self.docking_state == 0:
                self.docking_state = 1
                self.get_logger().info("🎯 Mengunci Bola Biru! Memulai pendekatan lurus...")

            if self.docking_state == 1:
                # ---> REVISI 4: LOG KALIBRASI <---
                self.get_logger().info(f"Mendekat pelan... Area Kamera: {self.last_box_msg.y:.0f} | Target Belok: {self.STOP_BOX_AREA}")

                if self.last_box_msg.y >= self.STOP_BOX_AREA:
                    self.docking_state = 2
                    self.pivot_start_time = curr_time
                    self.get_logger().info("🔄 FASE OMNI: Menggeser kapal menyamping ke dermaga!")
                else:
                    turn = self.get_fuzzy_steering(self.last_box_msg.x, max_error_val=320, inverted_output=False)
                    cmd.linear.x = 0.05 # Kecepatan dikurangi sedikit agar tidak menabrak frontal
                    cmd.linear.y = 0.0
                    cmd.angular.z = turn
                
                self.pub_cmd.publish(cmd)
                return

        # === PRIORITAS 2 & 3: VISION GATE & GPS (Sama seperti sebelumnya) ===
        ball_visible = (self.last_gate_error != 9999.0) and (curr_time - self.last_gate_time < 0.3)
        if ball_visible:
            turn = self.get_fuzzy_steering(-self.last_gate_error, max_error_val=320, inverted_output=False)
            speed = max(self.SPEED_NORMAL - (abs(turn) * 0.5), 0.4)
            cmd.linear.x = float(speed)
            cmd.linear.y = 0.0 
            cmd.angular.z = turn
            self.pub_cmd.publish(cmd)
        else:
            if self.gate_idx >= len(self.GATE_WAYPOINTS):
                self.navigate_gps_fuzzy(cmd, self.BOX_SEARCH_AREA, speed=0.6)
                return

            target = self.GATE_WAYPOINTS[self.gate_idx]
            dist = get_distance_meters(self.current_gps[0], self.current_gps[1], target[0], target[1])

            if dist < 3.0:
                self.get_logger().info(f"✅ WP {self.gate_idx+1} Sukses Lewat (Jarak: {dist:.2f}m). Berlanjut ke WP berikutnya.")
                self.gate_idx += 1
                self.last_gate_error = 9999.0
                return

            self.navigate_gps_fuzzy(cmd, target, speed=self.SPEED_SEARCH)
            
    def navigate_gps_fuzzy(self, cmd, target, speed=1.0):
        bearing = get_bearing(
            self.current_gps[0], self.current_gps[1], target[0], target[1]
        )
        heading_err = bearing - self.current_heading
        if heading_err > 180: heading_err -= 360
        if heading_err < -180: heading_err += 360

        turn = self.get_fuzzy_steering(heading_err, max_error_val=180.0, inverted_output=False)
        cmd.linear.x = float(speed)
        cmd.angular.z = turn
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = LogicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()