#!/usr/bin/env python3
# filepath: /home/gines/Escritorio/VAR_prac1/catkin_ws/src/genetic_nav/scripts/turtlebot_yolo_detector.py
# pip install ultralytics opencv-python cv_bridge
# sudo apt-get install ros-noetic-cv-bridge ros-noetic-image-transport

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class TurtlebotYOLODetector:
    def __init__(self):
        rospy.init_node('turtlebot_yolo_detector', anonymous=True)
        self.bridge = CvBridge()
        self.camera_topic = "/camera/rgb/image_raw"
        self.detection_pub = rospy.Publisher('/object_detection', String, queue_size=10)
        self.image_pub = rospy.Publisher('/detection_visualization', Image, queue_size=10)
        # Load YOLOv8 pre-trained on COCO
        self.model = YOLO('yolov8n.pt')
        # COCO class IDs for traffic cone and trash can
        self.target_classes = {39: 'traffic_cone', 0: 'person'}
        rospy.Subscriber(self.camera_topic, Image, self.image_callback)
        rospy.loginfo("Turtlebot YOLO detector initialized.")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"cv_bridge error: {e}")
            return

        results = self.model(cv_image, conf = 0.01)
        detected = []
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                if cls_id in self.target_classes:
                    label = self.target_classes[cls_id]
                    detected.append(label)
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0,255,0), 2)
                    cv2.putText(cv_image, f"{label} {conf:.2f}", (x1, y1-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                    rospy.loginfo(f"Detected: {label} ({conf:.2f}) at [{x1},{y1},{x2},{y2}]")
        # Publish detections
        if detected:
            self.detection_pub.publish(','.join(detected))
        # Optionally publish annotated image
        try:
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.image_pub.publish(img_msg)
        except Exception as e:
            rospy.logerr(f"cv_bridge error (publish): {e}")

if __name__ == '__main__':
    detector = TurtlebotYOLODetector()
    rospy.spin()