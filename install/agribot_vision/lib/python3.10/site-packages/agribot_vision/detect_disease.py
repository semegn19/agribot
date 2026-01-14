#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import csv
import os
import math

# TF2 Imports
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

os.environ['YOLO_OFFLINE'] = 'True'
from ultralytics import YOLO

class AgriVisionTFLogger(Node):
    def __init__(self):
        super().__init__('agribot_vision_node')
        
        # 1. YOLO Setup
        model_path = '/home/semegn/agribot_ws/src/agribot_vision/agribot_vision/best.pt' 
        self.model = YOLO(model_path)
        
        # 2. TF Setup (Replacing Odometry)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 3. Config & Map
        self.image_base_path = '/home/semegn/agribot_ws/src/agribot_gazebo/worlds/'
        self.log_file = '/home/semegn/agribot_ws/farm_health_report.csv'
        self.plant_map = {

            (2.0, 6.0): 'textures/healthy_1.jpg', (3.0, 6.0): 'textures/diseased_1.jpg',

            (4.0, 6.0): 'textures/healthy_2.jpg', (5.0, 6.0): 'textures/healthy_3.jpg',

            (6.0, 6.0): 'textures/diseased_2.jpg', (7.0, 6.0): 'textures/healthy_4.jpg',

            (2.0, 3.0): 'textures/healthy_5.jpg', (3.0, 3.0): 'textures/diseased_3.jpg',

            (4.0, 3.0): 'textures/healthy_6.jpg', (5.0, 3.0): 'textures/healthy_7.jpg',

            (6.0, 3.0): 'textures/diseased_4.jpg', (7.0, 3.0): 'textures/healthy_8.jpg',

            (2.0, -3.0): 'textures/healthy_9.jpg', (3.0, -3.0): 'textures/diseased_5.jpg',

            (4.0, -3.0): 'textures/healthy_10.jpg', (5.0, -3.0): 'textures/healthy_11.jpg',

            (6.0, -3.0): 'textures/diseased_6.jpg', (7.0, -3.0): 'textures/healthy_12.jpg',

            (2.0, -6.0): 'textures/healthy_13.jpg', (3.0, -6.0): 'textures/diseased_7.jpg',

            (4.0, -6.0): 'textures/healthy_14.jpg', (5.0, -6.0): 'textures/healthy_15.jpg',

            (6.0, -6.0): 'textures/diseased_8.jpg', (7.0, -6.0): 'textures/diseased_9.jpg'

        }

        self.subscription = self.create_subscription(Image, '/camera', self.image_callback, 10)
        self.bridge = CvBridge()
        self.robot_x, self.robot_y = 0.0, 0.0
        self.last_log_time = 0.0

        self.get_logger().info("TF-Based Vision Node Started. Tracking 'map' -> 'base_link'")

    def update_robot_pose(self):
        """Gets current coordinates from the TF tree instead of Odom topic."""
        try:
            # Look up the transform (same as what you did in terminal)
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.robot_x = t.transform.translation.x
            self.robot_y = t.transform.translation.y
            return True
        except TransformException as ex:
            return False

    def image_callback(self, msg):
        # 1. Get latest position from TF
        if not self.update_robot_pose():
            return # Wait until TF is available

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        active_injection = False
        
        # 2. Plant Injection Logic
        for (px, py), img_path in self.plant_map.items():
            dist = math.sqrt((self.robot_x - px)**2 + (self.robot_y - py)**2)
            if dist < 1.2:
                full_path = os.path.join(self.image_base_path, img_path)
                real_img = cv2.imread(full_path)
                if real_img is not None:
                    frame = cv2.resize(real_img, (640, 480))
                    active_injection = True
                    break

        # 3. YOLO Inference
        if active_injection:
            results = self.model.predict(source=frame, conf=0.4, verbose=False)
            for r in results:
                for box in r.boxes:
                    label = self.model.names[int(box.cls[0])]
                    conf = float(box.conf[0])
                    color = (0, 0, 255) if "diseas" in label.lower() else (0, 255, 0)
                    
                    # Draw
                    b = box.xyxy[0].cpu().numpy().astype(int)
                    cv2.rectangle(frame, (b[0], b[1]), (b[2], b[3]), color, 2)
                    cv2.putText(frame, f"{label} {conf:.2f}", (b[0], b[1]-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                    if color == (0, 0, 255):
                        self.save_to_log(label, conf)

        cv2.imshow("AgriBot TF-Vision System", frame)
        cv2.waitKey(1)

    def save_to_log(self, label, confidence):
        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self.last_log_time) > 3.0:
            self.last_log_time = now
            self.get_logger().warn(f"DISEASE DETECTED at [{self.robot_x:.2f}, {self.robot_y:.2f}]")
            with open(self.log_file, 'a') as f:
                csv.writer(f).writerow([round(self.robot_x, 2), round(self.robot_y, 2), label, round(confidence, 2), now])

def main():
    rclpy.init()
    node = AgriVisionTFLogger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
