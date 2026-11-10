#!/usr/bin/env python3
import time
import numpy as np
import cv2
import torch
import threading  # <-- Tambahan wajib untuk mengatasi lag kamera

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage, Image  # Tambahkan Image
from std_msgs.msg import Float64, Int32
from ultralytics import YOLO
from ultralytics import settings

# Matikan sinkronisasi dan auto-update Ultralytics agar tidak nge-lag di Jetson
settings.update({'sync': False})

# === OPTIMASI GPU / CUDA UNTUK JETSON ===
torch.backends.cudnn.enabled = True
torch.backends.cudnn.benchmark = True

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # Parameter: 0 = Lintasan A, 1 = Lintasan B
        self.declare_parameter('lintasan_mode', 1)
        self.lintasan_mode = self.get_parameter('lintasan_mode').get_parameter_value().integer_value

        self.get_logger().info(
            f"🚀 Vision Node Started | MODE AKTIF: LINTASAN {'B' if self.lintasan_mode == 1 else 'A'}"
        )

        # Publishers & Subscribers
        self.gate_pub = self.create_publisher(Float64, '/gate_error', 10)
        self.box_pub = self.create_publisher(Point, '/box_position', 10)
        self.debug_pub = self.create_publisher(CompressedImage, '/vision/debug/compressed', 10)
        self.rviz_pub = self.create_publisher(Image, '/vision/debug/image_raw', 10) 
        
        # Publisher untuk RViz2
        self.upload_pub = self.create_publisher(CompressedImage, '/upload/request', 10)
        self.mode_sub = self.create_subscription(Int32, '/change_lintasan_mode', self.mode_cb, 10)

        # === LOAD MODEL YOLO KE GPU ===
        model_path = '/home/aeronautika/Documents/ros2_ws/ros2_ws/src/asv26/asv26/lomba.engine'
        try:
            if torch.cuda.is_available():
                self.get_logger().info(f'🚀 CUDA Terdeteksi! Menggunakan GPU: {torch.cuda.get_device_name(0)}')
            else:
                self.get_logger().warn('⚠️ CUDA TIDAK TERDETEKSI! Jatuh kembali ke CPU.')

            self.model = YOLO(model_path, task='detect')
            self.get_logger().info('✅ Model YOLO TensorRT Sukses Dimuat!')
        except Exception as e:
            self.get_logger().error(f'❌ Gagal Memuat Model: {e}')
            self.model = None

        # === SETUP KAMERA UTAMA (CAM 1) ===
        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(1, cv2.CAP_V4L2)
            
        if not self.cap.isOpened():
            self.get_logger().error('❌ Gagal membuka kamera utama')
            raise RuntimeError('Camera utama tidak dapat dibuka')

        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.frame_center_x = 320
        self.get_logger().info('✅ Kamera utama (Cam 1) sukses dibuka')

        # === SETUP KAMERA SNAPSHOT (CAM 2) ===
        self.cam2 = cv2.VideoCapture('/dev/video2', cv2.CAP_V4L2)
        if not self.cam2.isOpened():
            self.cam2 = cv2.VideoCapture(1, cv2.CAP_V4L2)
            
        if self.cam2.isOpened():
            self.cam2.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.cam2.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cam2.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.get_logger().info('✅ Kamera kedua (Cam 2) siap sedia di background')
        else:
            self.get_logger().warn('⚠️ Kamera kedua gagal dibuka!')

        self.disp_w = 320
        self.disp_h = 240
        self.last_upload_time = 0
        self.UPLOAD_COOLDOWN = 5.0
        self.STOP_AREA_THRESHOLD = 20000

        self.running = True
        self.latest_frame = None
        self.latest_cam2_frame = None

        self.camera_thread = threading.Thread(target=self.camera_worker, daemon=True)
        self.camera_thread.start()

        self.prev_time = time.time()
        self.timer = self.create_timer(0.033, self.timer_callback)

        self.log_counter = 0

    def camera_worker(self):
        while self.running:
            if self.cap and self.cap.isOpened():
                ret1, frame1 = self.cap.read()
                if ret1:
                    self.latest_frame = frame1
            
            if self.cam2 and self.cam2.isOpened():
                ret2, frame2 = self.cam2.read()
                if ret2:
                    self.latest_cam2_frame = frame2
            
            time.sleep(0.005)

    def mode_cb(self, msg):
        self.lintasan_mode = msg.data
        mode_str = 'B' if self.lintasan_mode == 1 else 'A'
        self.get_logger().info(f"🔄 Lintasan berganti dinamis ke: {mode_str}")

    def take_snapshot_cam2(self):
        if self.latest_cam2_frame is not None:
            return self.latest_cam2_frame.copy()
        return None

    def fuzzy_membership_x_5mf(self, x, max_x=640):
        norm_x = (x - (max_x / 2.0)) / (max_x / 2.0)
        norm_x = max(min(norm_x, 1.0), -1.0)
        if norm_x <= -0.6: ll = 1.0
        elif -0.6 < norm_x < -0.2: ll = (-0.2 - norm_x) / 0.4
        else: ll = 0.0

        if -0.6 < norm_x <= -0.2: ls = (norm_x + 0.6) / 0.4
        elif -0.2 < norm_x < 0.0: ls = (0.0 - norm_x) / 0.2
        else: ls = 0.0

        if -0.2 < norm_x <= 0.0: z = (norm_x + 0.2) / 0.2
        elif 0.0 < norm_x < 0.2: z = (0.2 - norm_x) / 0.2
        else: z = 0.0

        if 0.0 < norm_x <= 0.2: rs = norm_x / 0.2
        elif 0.2 < norm_x < 0.6: rs = (0.6 - norm_x) / 0.4
        else: rs = 0.0

        if norm_x >= 0.6: rl = 1.0
        elif 0.2 < norm_x < 0.6: rl = (norm_x - 0.2) / 0.4
        else: rl = 0.0
        return ll, ls, z, rs, rl

    def get_fuzzy_offset(self, ball_x, is_red=True):
        ll, ls, z, rs, rl = self.fuzzy_membership_x_5mf(ball_x)
        OFFSET_LARGE = 280
        OFFSET_MED_LARGE = 220
        OFFSET_MEDIUM = 180
        OFFSET_MED_SMALL = 140
        OFFSET_SMALL = 100

        if self.lintasan_mode == 1:
            if is_red: w1,out1=ll,OFFSET_LARGE; w2,out2=ls,OFFSET_MED_LARGE; w3,out3=z,OFFSET_MEDIUM; w4,out4=rs,OFFSET_MED_SMALL; w5,out5=rl,OFFSET_SMALL
            else: w1,out1=ll,OFFSET_SMALL; w2,out2=ls,OFFSET_MED_SMALL; w3,out3=z,OFFSET_MEDIUM; w4,out4=rs,OFFSET_MED_LARGE; w5,out5=rl,OFFSET_LARGE
        else:
            if is_red: w1,out1=ll,OFFSET_SMALL; w2,out2=ls,OFFSET_MED_SMALL; w3,out3=z,OFFSET_MEDIUM; w4,out4=rs,OFFSET_MED_LARGE; w5,out5=rl,OFFSET_LARGE
            else: w1,out1=ll,OFFSET_LARGE; w2,out2=ls,OFFSET_MED_LARGE; w3,out3=z,OFFSET_MEDIUM; w4,out4=rs,OFFSET_MED_SMALL; w5,out5=rl,OFFSET_SMALL

        total_weight = w1 + w2 + w3 + w4 + w5
        if total_weight == 0: return 180.0
        return (w1 * out1 + w2 * out2 + w3 * out3 + w4 * out4 + w5 * out5) / total_weight

    def timer_callback(self):
        if self.latest_frame is None:
            return
            
        frame = self.latest_frame.copy()
        debug_frame = frame.copy()

        curr_time = time.time()
        time_diff = curr_time - self.prev_time
        fps = 1.0 / time_diff if time_diff > 0 else 0.0
        self.prev_time = curr_time

        mode_str = "B (Green L, Red R)" if self.lintasan_mode == 1 else "A (Red L, Green R)"
        cv2.putText(debug_frame, f'FPS: {int(fps)} | MODE: {mode_str}', (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        red_ball_center = None
        green_ball_center = None
        target_box = None
        max_box_area = 0
        box_type = 0
        target_class_name = None  # <-- TAMBAHAN: Untuk merekam nama objek

        if self.model is not None:
            results = self.model(frame, conf=0.25, iou=0.45, verbose=False, device='cuda:0')
            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                area = (x2 - x1) * (y2 - y1)
                cls_id = int(box.cls[0])
                
                cls_name = self.model.names[cls_id] if cls_id in self.model.names else str(cls_id)
                name_lower = cls_name.lower()
                color = (0, 255, 0)
                
                 # ========================================================
                # BLOK KLASIFIKASI KELAS YOLO
                # ========================================================
                if name_lower == 'bola-merah':
                    color = (0, 0, 255)
                    if red_ball_center is None or cy > red_ball_center[1]:
                        red_ball_center = (cx, cy)
                        
                elif name_lower == 'bola-hijau':
                    color = (0, 255, 0)
                    if green_ball_center is None or cy > green_ball_center[1]:
                        green_ball_center = (cx, cy)
                        
                elif name_lower == 'bola-biru':
                    conf_score = float(box.conf[0])
                    if conf_score < 0.65:  # Harus minimal 65% yakin itu bola biru
                        continue           
                    
                    color = (255, 0, 0)  # BGR format untuk biru
                    if area > max_box_area:
                        max_box_area = area
                        target_box = (cx, cy)
                        box_type = 2  # Kode 2 dibaca logic_node untuk docking
                        target_class_name = 'bola-biru' # <-- TAMBAHKAN INI
                      
                elif name_lower == 'kotak-hijau':
                    # ========================================================
                    # TAMBAHAN: Filter Confidence Khusus Kotak Hijau
                    # ========================================================
                    conf_score = float(box.conf[0])
                    if conf_score < 0.30:  # Harus minimal 90% yakin itu kotak hijau
                        continue
                    # ========================================================
                    
                    if area > max_box_area:
                        max_box_area = area
                        target_box = (cx, cy)
                        box_type = 1
                        color = (0, 255, 0)
                        target_class_name = 'kotak-hijau' # <-- TAMBAHKAN INI
                        
                elif name_lower == 'kotak-biru':
                    # ========================================================
                    # TAMBAHAN: Filter Confidence Khusus Kotak Biru
                    # ========================================================
                    conf_score = float(box.conf[0])
                    if conf_score < 0.30:  # Harus minimal 80% yakin itu kotak biru
                        continue
                    # ========================================================
                    
                    if area > max_box_area:
                        max_box_area = area
                        target_box = (cx, cy)
                        box_type = 2
                        color = (255, 0, 0)
                        target_class_name = 'kotak-biru' # <-- TAMBAHKAN INI
                else:
                    color = (255, 0, 255) # Warna default untuk objek tak dikenal
                # ========================================================

                cv2.rectangle(debug_frame, (x1, y1), (x2, y2), color, 2)
                label = f"{cls_name} {float(box.conf[0]):.2f}"
                cv2.putText(debug_frame, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        gate_err = Float64()
        gate_err.data = 9999.0
        target_x = None

        status_bola = "TIDAK ADA BOLA"
        status_belok = "LURUS (Mencari...)"

        # ==========================================
        # LOGIKA PERGERAKAN KAPAL (REVISI MENGHINDAR)
        # ==========================================

        if red_ball_center and green_ball_center:
            target_x = (red_ball_center[0] + green_ball_center[0]) // 2
            cv2.line(debug_frame, red_ball_center, green_ball_center, (0, 255, 255), 2)
            cv2.putText(debug_frame, 'MODE: 2 BALLS', (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            status_bola = "MERAH & HIJAU (Center Point)"

        elif red_ball_center and not green_ball_center:
            fuzzy_offset = self.get_fuzzy_offset(red_ball_center[0], is_red=True)
            abs_offset = abs(int(fuzzy_offset))
            
            if self.lintasan_mode == 0:
                gate_err.data = -float(abs_offset) 
                arah_target = "Kanan"
            else:
                gate_err.data = float(abs_offset)  
                arah_target = "Kiri"
                
            cv2.circle(debug_frame, red_ball_center, 5, (0,0,255), -1)
            cv2.putText(debug_frame, f"MENGHINDAR {arah_target.upper()} ({abs_offset}px)", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            status_bola = f"MERAH SAJA (Menghindar {arah_target})"

        elif green_ball_center and not red_ball_center:
            fuzzy_offset = self.get_fuzzy_offset(green_ball_center[0], is_red=False)
            abs_offset = abs(int(fuzzy_offset))
            
            if self.lintasan_mode == 0:
                gate_err.data = float(abs_offset) 
                arah_target = "Kiri"
            else:
                gate_err.data = -float(abs_offset) 
                arah_target = "Kanan"
                
            cv2.circle(debug_frame, green_ball_center, 5, (0,255,0), -1)
            cv2.putText(debug_frame, f"MENGHINDAR {arah_target.upper()} ({abs_offset}px)", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            status_bola = f"HIJAU SAJA (Menghindar {arah_target})"

        if target_x is not None:
            gate_err.data = float(self.frame_center_x - target_x)
            cv2.circle(debug_frame, (int(target_x), 240), 10, (255, 255, 0), -1)

        if gate_err.data > 15.0 and gate_err.data != 9999.0:
            status_belok = f"⬅️ BELOK KIRI  (Error: +{gate_err.data:.1f})"
        elif gate_err.data < -15.0:
            status_belok = f"➡️ BELOK KANAN (Error: {gate_err.data:.1f})"
        elif gate_err.data != 9999.0:
            status_belok = f"⬆️ LURUS       (Error: {gate_err.data:.1f})"

        self.gate_pub.publish(gate_err)

        self.log_counter += 1
        if self.log_counter % 15 == 0:
            self.get_logger().info(f"👀 Deteksi: {status_bola.ljust(35)} | ⛵ Arah: {status_belok}")

        if target_box:
            box_msg = Point()
            box_msg.x = float(target_box[0] - self.frame_center_x)
            box_msg.y = float(max_box_area)
            box_msg.z = float(box_type)
            self.box_pub.publish(box_msg)

            # <-- TAMBAHKAN SYARAT IF INI AGAR BOLA BIRU TIDAK TERUPLOAD -->
            if target_class_name in ['kotak-biru', 'kotak-hijau']:
                
                if max_box_area > self.STOP_AREA_THRESHOLD and (time.time() - self.last_upload_time > self.UPLOAD_COOLDOWN):
                    self.last_upload_time = time.time()
                    msg = CompressedImage()
                    msg.format = 'jpeg'
                    
                    if box_type == 2:
                        msg.header.frame_id = 'blue'
                        cam2_frame = self.take_snapshot_cam2()
                        frame_to_upload = cam2_frame if cam2_frame is not None else debug_frame
                    else:
                        msg.header.frame_id = 'green'
                        frame_to_upload = debug_frame

                    if frame_to_upload is not None:
                        success, encoded_img = cv2.imencode('.jpg', frame_to_upload)
                        if success:
                            msg.data = np.array(encoded_img).tobytes()
                            self.upload_pub.publish(msg)
                            self.get_logger().info(f"🚀 UPLOAD FOTO BERHASIL: {msg.header.frame_id}")

        resized_debug = cv2.resize(debug_frame, (self.disp_w, self.disp_h), interpolation=cv2.INTER_NEAREST)
        cv2.line(resized_debug, (self.disp_w // 2, 0), (self.disp_w // 2, self.disp_h), (255, 255, 0), 1)

        # 1. Publish Gambar yang dikompresi ke Telemetri (Tetap ada)
        msg_comp = CompressedImage()
        msg_comp.format = 'jpeg'
        msg_comp.header.stamp = self.get_clock().now().to_msg()
        success, encoded_img = cv2.imencode('.jpg', resized_debug, [int(cv2.IMWRITE_JPEG_QUALITY), 40])
        
        if success:
            msg_comp.data = np.array(encoded_img).tobytes()
            self.debug_pub.publish(msg_comp)

        # 2. Publish Gambar format Image murni untuk RViz2 (TANPA cv_bridge)
        try:
            rviz_msg = Image()
            rviz_msg.header.stamp = self.get_clock().now().to_msg()
            rviz_msg.header.frame_id = "camera_link"
            
            # Isi data gambar secara manual (bypass library cv_bridge yang crash di Jetson)
            rviz_msg.height = resized_debug.shape[0]
            rviz_msg.width = resized_debug.shape[1]
            rviz_msg.encoding = "bgr8"
            rviz_msg.is_bigendian = 0
            rviz_msg.step = resized_debug.shape[1] * 3  # Lebar gambar dikali 3 channel (B, G, R)
            rviz_msg.data = resized_debug.tobytes()     # Konversi NumPy array ke tipe data bytes
            
            self.rviz_pub.publish(rviz_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing ke RViz manual: {e}")
            
        # Matikan cv2.imshow agar tidak membebani CPU saat dijalankan di atas kapal
        #cv2.imshow('Vision Kapal Varuna', resized_debug)
        #cv2.waitKey(1)

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'camera_thread'):
            self.camera_thread.join(timeout=1.0)
            
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            
        if hasattr(self, 'cam2') and self.cam2.isOpened():
            self.cam2.release()
            
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()