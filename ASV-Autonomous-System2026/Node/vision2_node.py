#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time
from std_msgs.msg import Float64, Int32
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO
import torch

# Optimasi CUDA jika tersedia
torch.backends.cudnn.enabled = False
torch.backends.cudnn.benchmark = False

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.get_logger().info('Vision Node Multi-Lintasan dengan Fuzzy Logic Started')

        # === PARAMETER LINTASAN ===
        # 0 = Lintasan A (Merah Kiri, Hijau Kanan)
        # 1 = Lintasan B (Merah Kanan, Hijau Kiri)
        self.declare_parameter('lintasan_mode', 0)
        self.lintasan_mode = self.get_parameter('lintasan_mode').value
        self.get_logger().info(f"▶️ MODE AKTIF: LINTASAN {'A' if self.lintasan_mode == 0 else 'B'}")

        # === PUBLISHERS & SUBSCRIBERS ===
        self.gate_pub = self.create_publisher(Float64, '/gate_error', 10)
        self.box_pub = self.create_publisher(Point, '/box_position', 10)
        self.debug_pub = self.create_publisher(CompressedImage, '/vision/debug/compressed', 10)
        self.upload_pub = self.create_publisher(CompressedImage, '/upload/request', 10)
        self.mode_sub = self.create_subscription(Int32, '/change_lintasan_mode', self.mode_cb, 10)

        # === LOAD MODEL YOLO ===
        model_path = '/home/aeronautika/ros2_ws/src/asv26/asv26/best (2).pt'
        try:
            self.model = YOLO(model_path)
            self.get_logger().info("✅ Model YOLO Sukses Dimuat!")
        except Exception as e:
            self.get_logger().error(f"❌ Gagal Memuat Model: {e}")
            self.model = None

        # === SETUP KAMERA === 
        self.cap = cv2.VideoCapture('/dev/video0') 

        if not self.cap.isOpened(): 
            self.get_logger().error("Gagal membuka /dev/video0") 
            raise RuntimeError("Camera tidak dapat dibuka") 
    
        ret, frame = self.cap.read() 

        self.get_logger().info(f"Camera opened={self.cap.isOpened()} ret={ret}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.frame_center_x = 320
        
        self.last_upload_time = 0
        self.UPLOAD_COOLDOWN = 5.0 
        self.STOP_AREA_THRESHOLD = 20000

        # Loop pemrosesan gambar 30 FPS
        self.timer = self.create_timer(0.033, self.timer_callback)

    def mode_cb(self, msg):
        self.lintasan_mode = msg.data
        self.get_logger().info(f"🔄 Lintasan berganti dinamis ke: {'A' if self.lintasan_mode == 0 else 'B'}")

    # ================= FUZZY ADAPTIF MULTI-LINTASAN =================
    def fuzzy_membership_x(self, x):
        if x <= 160: left = 1.0
        elif 160 < x < 320: left = (320 - x) / 160
        else: left = 0.0

        if 160 < x <= 320: center = (x - 160) / 160
        elif 320 < x < 480: center = (480 - x) / 160
        else: center = 0.0

        if x >= 480: right = 1.0
        elif 320 < x < 480: right = (x - 320) / 160
        else: right = 0.0
        return left, center, right

    def get_fuzzy_offset(self, ball_x, is_red=True):
        mu_left, mu_center, mu_right = self.fuzzy_membership_x(ball_x)
        OFFSET_LARGE = 270
        OFFSET_MEDIUM = 190
        OFFSET_SMALL = 110

        if self.lintasan_mode == 0:
            # --- LINTASAN A (Merah Kiri, Hijau Kanan) ---
            if is_red: # Merah di Kiri -> Geser target ke Kanan (+)
                w1, out1 = mu_left, OFFSET_LARGE
                w2, out2 = mu_center, OFFSET_MEDIUM
                w3, out3 = mu_right, OFFSET_SMALL
            else:      # Hijau di Kanan -> Geser target ke Kiri (-)
                w1, out1 = mu_left, OFFSET_SMALL
                w2, out2 = mu_center, OFFSET_MEDIUM
                w3, out3 = mu_right, OFFSET_LARGE
        else:
            # --- LINTASAN B (Merah Kanan, Hijau Kiri) ---
            if is_red: # Merah di Kanan -> Geser target ke Kiri (-)
                w1, out1 = mu_left, OFFSET_SMALL
                w2, out2 = mu_center, OFFSET_MEDIUM
                w3, out3 = mu_right, OFFSET_LARGE
            else:      # Hijau di Kiri -> Geser target ke Kanan (+)
                w1, out1 = mu_left, OFFSET_LARGE
                w2, out2 = mu_center, OFFSET_MEDIUM
                w3, out3 = mu_right, OFFSET_SMALL

        total_weight = w1 + w2 + w3
        if total_weight == 0: return 190.0
        return (w1 * out1 + w2 * out2 + w3 * out3) / total_weight
    # ======================================================================

    #def take_snapshot_cam2(self):
        #cam2 = cv2.VideoCapture(1, cv2.CAP_V4L2)
        #if not cam2.isOpened(): return None
        #cam2.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        #cam2.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        #frame_result = None
        #for _ in range(5): ret, frame_result = cam2.read()
        #cam2.release()
        #return frame_result

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret: return
        debug_frame = frame.copy()
        
        red_ball_center = None
        green_ball_center = None
        target_box = None
        max_box_area = 0
        box_type = 0

        if self.model is not None:
            results = self.model(frame, conf=0.25, iou=0.45, verbose=False, device=0)
            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = (x1+x2)//2, (y1+y2)//2
                area = (x2-x1)*(y2-y1)
                cls_id = int(box.cls[0])
                name_lower = self.model.names[cls_id].lower()

                if 'ball' in name_lower or 'bola' in name_lower:
                    if 'red' in name_lower or 'merah' in name_lower:
                        if red_ball_center is None or cy > red_ball_center[1]: red_ball_center = (cx, cy)
                    elif 'green' in name_lower or 'hijau' in name_lower:
                        if green_ball_center is None or cy > green_ball_center[1]: green_ball_center = (cx, cy)
                elif 'box' in name_lower or 'kotak' in name_lower:
                    if area > max_box_area:
                        max_box_area = area
                        target_box = (cx, cy)
                        if 'green' in name_lower or 'hijau' in name_lower: box_type = 1
                        elif 'blue' in name_lower or 'biru' in name_lower: box_type = 2

        gate_err = Float64(); gate_err.data = 9999.0
        target_x = None

        # Kasus 1: Dua Bola
        if red_ball_center and green_ball_center:
            target_x = (red_ball_center[0] + green_ball_center[0]) // 2
            cv2.putText(debug_frame, "2 BALLS CENTER", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        # Kasus 2: Hanya Merah
        elif red_ball_center and not green_ball_center:
            fuzzy_offset = self.get_fuzzy_offset(red_ball_center[0], is_red=True)
            target_x = red_ball_center[0] + int(fuzzy_offset) if self.lintasan_mode == 0 else red_ball_center[0] - int(fuzzy_offset)
            cv2.putText(debug_frame, f"FUZZY MERAH: {'+' if self.lintasan_mode==0 else '-'}{int(fuzzy_offset)}px", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Kasus 3: Hanya Hijau
        elif green_ball_center and not red_ball_center:
            fuzzy_offset = self.get_fuzzy_offset(green_ball_center[0], is_red=False)
            target_x = green_ball_center[0] - int(fuzzy_offset) if self.lintasan_mode == 0 else green_ball_center[0] + int(fuzzy_offset)
            cv2.putText(debug_frame, f"FUZZY HIJAU: {'-' if self.lintasan_mode==0 else '+'}{int(fuzzy_offset)}px", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        if target_x is not None:
            gate_err.data = float(self.frame_center_x - target_x)
            cv2.circle(debug_frame, (int(target_x), 240), 10, (0, 255, 255), -1)
            
        self.gate_pub.publish(gate_err)

        if target_box:
            box_msg = Point()
            box_msg.x = float(target_box[0] - self.frame_center_x)
            box_msg.y = float(max_box_area)
            box_msg.z = float(box_type)
            self.box_pub.publish(box_msg)
            
            if max_box_area > self.STOP_AREA_THRESHOLD and (time.time() - self.last_upload_time > self.UPLOAD_COOLDOWN):
                self.last_upload_time = time.time()
                msg = CompressedImage(); msg.format = "jpeg"
                if box_type == 2:
                    msg.header.frame_id = "blue"
                    #frame_to_upload = self.take_snapshot_cam2() or debug_frame
                else:
                    msg.header.frame_id = "green"
                    frame_to_upload = debug_frame

                if frame_to_upload is not None:
                    success, encoded_img = cv2.imencode('.jpg', frame_to_upload)
                    if success:
                        msg.data = np.array(encoded_img).tobytes()
                        self.upload_pub.publish(msg)

        # Publish Debug Image
        msg = CompressedImage(); msg.format = "jpeg"
        success, encoded_img = cv2.imencode('.jpg', debug_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        if success:
            msg.data = np.array(encoded_img).tobytes()
            self.debug_pub.publish(msg)

        cv2.line(debug_frame, (self.frame_center_x, 0), (self.frame_center_x, 480), (255, 255, 0), 1)
        cv2.imshow("Vision Debug", debug_frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args); node = VisionNode(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()