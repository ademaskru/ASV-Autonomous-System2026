#!/usr/bin/env python3
import os
import time
import math
import threading
from datetime import datetime
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, CompressedImage
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from supabase import create_client, Client
from rclpy.qos import qos_profile_sensor_data

# ==========================================
# === 1. KONFIGURASI SUPABASE ===
# ==========================================
SUPABASE_URL = "https://usljlghwqdfprvmbnvlv.supabase.co"
SUPABASE_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InVzbGpsZ2h3cWRmcHJ2bWJudmx2Iiwicm9sZSI6ImFub24iLCJpYXQiOjE3ODQ5MDc0MzMsImV4cCI6MjEwMDQ4MzQzM30.lVS9iDg08k7sww0T7xO1Yw4xH0bYVEuAE1PyDXZUvtE"

# ==========================================
# === 2. KONFIGURASI MONITORING LINTASAN B ===
# ==========================================
START_OFFSET_X = 3.5  # <-- Offset khusus Lintasan B
START_OFFSET_Y = 5.0
UKURAN_KOTAK_METER = 6.0
JUMLAH_SAMPEL_ORIGIN = 30
INTERVAL_UPLOAD_DATA = 2.0  # detik

# ==========================================
# === 3. FITUR KONTROL ARAH & VISUAL ===
# ==========================================
# Ubah AUTO_KALIBRASI_HEADING ke False agar kodingan menggunakan orientasi manual
AUTO_KALIBRASI_HEADING = False  

# Tentukan sendiri arah "MAJU" lintasan (hanya aktif jika AUTO_KALIBRASI_HEADING = False)
# 0   = Utara | 90  = Timur | 180 = Selatan | 270 = Barat
MANUAL_HEADING_DEG = 100.0      

INVERT_Y_AXIS = False           # True = Membalik sumbu Y agar tidak terlihat mundur di web

class SupabaseUploadNode(Node):
    def __init__(self):
        super().__init__('supabase_upload_node_b')
        self.get_logger().info("🚀 Memulai Supabase Telemetry Node B (ROS 2)...")

        # --- 1. INISIALISASI SUPABASE ---
        try:
            self.supabase: Client = create_client(SUPABASE_URL, SUPABASE_KEY)
            self.get_logger().info("✅ Supabase Connected!")
        except Exception as e:
            self.get_logger().error(f"❌ Gagal koneksi Supabase: {e}")
            exit(1)

        # ===== Kalibrasi Orientasi Arena =====
        self.arena_heading = None      # heading saat start
        self.heading_set = False
        
        # --- 2. VARIABEL STATUS & CACHE ---
        self.origin_lat = None
        self.origin_lon = None
        self.samples_lat = []
        self.samples_lon = []
        self.origin_set = False

        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_heading = 0.0
        self.current_sog = 0.0  # m/s
        self.gps_received = False

        # --- 3. ROS 2 SUBSCRIBERS ---
        self.gps_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, qos_profile_sensor_data)
        self.heading_sub = self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.heading_callback, qos_profile_sensor_data)
        self.vel_sub = self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local', self.vel_callback, qos_profile_sensor_data)
        self.image_req_sub = self.create_subscription(CompressedImage, '/upload/request', self.upload_request_cb, 10)

        # --- 4. ROS 2 TIMER ---
        self.upload_timer = self.create_timer(INTERVAL_UPLOAD_DATA, self.upload_timer_callback)

    # =========================================
    # LOGIKA PENGARAH MATA ANGIN OTOMATIS
    # =========================================
    def snap_to_cardinal(self, heading):
        h = heading % 360
        if h >= 315 or h < 45: return 0.0      # Utara
        elif 45 <= h < 135: return 90.0        # Timur
        elif 135 <= h < 225: return 180.0      # Selatan
        else: return 270.0                     # Barat

    def get_cardinal_name(self, heading):
        if heading == 0.0: return "Utara"
        elif heading == 90.0: return "Timur"
        elif heading == 180.0: return "Selatan"
        elif heading == 270.0: return "Barat"
        return "Tidak Diketahui"

    # =========================================
    # CALLBACK SENSOR (MAVROS)
    # =========================================
    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.gps_received = True

        if not self.origin_set:
            self.samples_lat.append(self.current_lat)
            self.samples_lon.append(self.current_lon)
            
            if len(self.samples_lat) % 10 == 0:
                self.get_logger().info(f"📍 Kalibrasi Origin: {len(self.samples_lat)}/{JUMLAH_SAMPEL_ORIGIN}")

            if len(self.samples_lat) >= JUMLAH_SAMPEL_ORIGIN:
                self.origin_lat = sum(self.samples_lat) / len(self.samples_lat)
                self.origin_lon = sum(self.samples_lon) / len(self.samples_lon)
                self.origin_set = True
                self.get_logger().info(f"✅ ORIGIN SET: Lat {self.origin_lat:.6f}, Lon {self.origin_lon:.6f}")

    def heading_callback(self, msg):
        self.current_heading = float(msg.data)
        
        if self.origin_set and not self.heading_set:
            if AUTO_KALIBRASI_HEADING:
                self.arena_heading = self.snap_to_cardinal(self.current_heading)
                arah = self.get_cardinal_name(self.arena_heading)
                self.get_logger().info(f"🧭 Arena AUTO-TERDETEKSI Menghadap: {arah} ({self.arena_heading:.2f}°)")
            else:
                self.arena_heading = MANUAL_HEADING_DEG % 360.0
                
                # --- LOG INFORMASI ORIENTASI MANUAL ---
                self.get_logger().info("==========================================")
                self.get_logger().info("🧭 ORIENTASI LINTASAN B MANUAL AKTIF")
                self.get_logger().info(f"🧭 Arah MAJU lintasan = {self.arena_heading:.2f}°")
                self.get_logger().info("0°=Utara | 90°=Timur | 180°=Selatan | 270°=Barat")
                self.get_logger().info("==========================================")
            
            self.heading_set = True

    def vel_callback(self, msg):
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        self.current_sog = math.sqrt(vx**2 + vy**2)

    # =========================================
    # LOGIKA HITUNG POSISI MATRIKS
    # =========================================
    def gps_to_xy(self, lat, lon):
        if not self.origin_set: return 0.0, 0.0
        R = 6371000
        lat_rad = math.radians(lat); lon_rad = math.radians(lon)
        org_lat_rad = math.radians(self.origin_lat); org_lon_rad = math.radians(self.origin_lon)

        x = R * (lon_rad - org_lon_rad) * math.cos(org_lat_rad)
        y = R * (lat_rad - org_lat_rad)
        return x, y

    def rotate_xy(self, x_gps, y_gps):
        if not self.heading_set:
            return x_gps, y_gps

        theta = math.radians(self.arena_heading)
        rot_y = y_gps * math.cos(theta) + x_gps * math.sin(theta)
        rot_x = x_gps * math.cos(theta) - y_gps * math.sin(theta)
        
        return rot_x, rot_y

    def get_position_label(self, x, y):
        map_x = ['A', 'B', 'C', 'D', 'E']
        map_y = ['1', '2', '3', '4', '5']
        idx_x = int(x // UKURAN_KOTAK_METER)
        idx_y = int(y // UKURAN_KOTAK_METER)
        idx_x = max(0, min(idx_x, 4)); idx_y = max(0, min(idx_y, 4))
        return f"{map_x[idx_x]}{map_y[idx_y]}"

    # =========================================
    # UPLOAD TELEMETRY (DIPANGGIL OLEH TIMER)
    # =========================================
    def upload_timer_callback(self):
        if not self.gps_received or not self.origin_set or not self.heading_set:
            return

        rel_x, rel_y = self.gps_to_xy(self.current_lat, self.current_lon)
        rot_x, rot_y = self.rotate_xy(rel_x, rel_y)

        # Inversi Sumbu Y jika INVERT_Y_AXIS = True (memperbaiki grafik web terbalik)
        y_final_rot = -rot_y if INVERT_Y_AXIS else rot_y

        final_x = START_OFFSET_X + rot_x
        final_y = START_OFFSET_Y + y_final_rot

        pos_label = self.get_position_label(final_x, final_y)
        waktu_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        lat_dir = "S" if self.current_lat < 0 else "N"
        lon_dir = "E" if self.current_lon > 0 else "W"
        coord_str = f"( {lat_dir} {abs(self.current_lat):.6f} {lon_dir} {abs(self.current_lon):.6f} )"
        
        sog_knots = self.current_sog * 1.94384

        upload_data = {
            "cog": float(self.current_heading),
            "heading": float(self.current_heading),
            "sog": round(sog_knots, 2),
            "score": 0,
            "position": pos_label,
            "koordinat": coord_str,
            "geotag": f"{waktu_str}, {coord_str}",
            "waktu": waktu_str,
            "x": round(final_x, 2),
            "y": round(final_y, 2)
        }

        threading.Thread(target=self.push_data_worker, args=(upload_data,), daemon=True).start()

    def push_data_worker(self, data):
        try:
            self.supabase.table("mission_logs").insert(data).execute()
            self.get_logger().debug("✅ Data telemetry B ter-upload")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Gagal Upload Data: {e}")

    # =========================================
    # UPLOAD FOTO (Dari vision_node)
    # =========================================
    def upload_request_cb(self, msg: CompressedImage):
        box_type = msg.header.frame_id 
        image_bytes = msg.data 
        
        timestamp = datetime.now().strftime("%H%M%S")
        filename = f"{box_type}_{timestamp}.jpg"

        self.get_logger().info(f"📸 Menerima request upload foto: {filename}")
        threading.Thread(target=self.upload_image_worker, args=(image_bytes, filename), daemon=True).start()

    def upload_image_worker(self, image_bytes, cloud_filename):
        local_filename = f"/tmp/{cloud_filename}"
        try:
            with open(local_filename, 'wb') as f:
                f.write(image_bytes)

            with open(local_filename, 'rb') as f:
                self.supabase.storage.from_("mission_images").upload(
                    file=f,
                    path=cloud_filename,
                    file_options={"content-type": "image/jpeg"}
                )
            self.get_logger().info(f"✅ FOTO TER-UPLOAD KE CLOUD: {cloud_filename}")
        except Exception as e:
            self.get_logger().error(f"❌ Gagal Upload Foto: {e}")
        finally:
            if os.path.exists(local_filename):
                os.remove(local_filename)

def main(args=None):
    rclpy.init(args=args)
    node = SupabaseUploadNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Program Dihentikan")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()