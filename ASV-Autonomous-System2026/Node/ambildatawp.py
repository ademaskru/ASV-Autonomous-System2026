#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import sys
import threading

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')
        
        self.TARGET_SAMPLES = 30 
        self.FILENAME = "saved_waypoints.txt"
        
        # PERBAIKAN TOPIC: Menggunakan /mavros/ sesuai output ros2 topic list kamu
        self.TOPIC_NAME = '/mavros/global_position/global'
        
        self.get_logger().info("--- WAYPOINT LOGGER (MAVROS COMPATIBLE) ---")
        self.get_logger().info(f"Mendengarkan topik: {self.TOPIC_NAME}")
        self.get_logger().info(f"Menyimpan ke: {self.FILENAME}")
        
        # PERBAIKAN QoS: MAVROS memerlukan BEST_EFFORT agar cocok di ROS 2
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            NavSatFix, 
            self.TOPIC_NAME, 
            self.listener_callback, 
            qos_profile)
        
        self.current_lat = None
        self.current_lon = None
        self.recording = False
        self.samples = []
        self.wp_counter = 0
        self.gps_received_once = False

        with open(self.FILENAME, "w") as f:
            f.write("# Daftar Waypoint\n")

    def listener_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        
        if not self.gps_received_once:
            self.get_logger().info(f"✅ TERHUBUNG KE MAVROS! GPS: Lat {msg.latitude:.5f}, Lon {msg.longitude:.5f}")
            self.get_logger().info("Tekan [ENTER] di terminal ini untuk merekam titik...")
            self.gps_received_once = True
        
        if self.recording:
            self.samples.append((msg.latitude, msg.longitude))
            sys.stdout.write(f"\r⏳ Sampling... {len(self.samples)}/{self.TARGET_SAMPLES}")
            sys.stdout.flush()

    def record_point(self):
        if not self.gps_received_once:
            print("\n⚠️ BELUM ADA DATA GPS! Tunggu sampai MAVROS terhubung dan GPS Lock 3D.")
            return

        print(f"\n\n--- Merekam Waypoint #{self.wp_counter + 1} ---")
        self.samples = []
        self.recording = True
        
        while len(self.samples) < self.TARGET_SAMPLES and rclpy.ok():
            pass
        
        self.recording = False
        
        avg_lat = np.mean([s[0] for s in self.samples])
        avg_lon = np.mean([s[1] for s in self.samples])
        
        line = f"    ({avg_lat:.8f}, {avg_lon:.8f}),  # WP {self.wp_counter + 1}\n"
        
        with open(self.FILENAME, "a") as f:
            f.write(line)
            
        print(f"\n✅ TITIK TERSIMPAN: {line.strip()}")
        self.wp_counter += 1

def input_thread_func(node):
    while rclpy.ok():
        try:
            input()
            node.record_point()
        except (EOFError, KeyboardInterrupt):
            break

def main():
    rclpy.init()
    node = WaypointLogger()
    
    input_thread = threading.Thread(target=input_thread_func, args=(node,), daemon=True)
    input_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\nSelesai. Cek file {node.FILENAME}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()