import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # Subscriber (Input Video)
        qos_profile_sub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            Image,
            '/front_camera',
            self.listener_callback,
            qos_profile_sub)
        
        self.get_logger().info("Waiting for data from /front_camera...")

        # Publishers
        self.image_publisher_ = self.create_publisher(Image, '/yolo_result', 10)
        self.coord_publisher_ = self.create_publisher(String, '/yolo_target_coord', 10)

        self.bridge = CvBridge()
        self.model = YOLO("final_test.pt")
        self.get_logger().info("YOLO Node Initialized. Publishing Image and Coordinates.")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            img_height, img_width = cv_image.shape[:2]
            
            # Run Inference
            results = self.model(cv_image, verbose=False, conf=0.75)
            annotated_frame = results[0].plot()

            det_count = len(results[0].boxes)
            all_objects_data = []

            if det_count > 0:
                grouped_boxes = {}

                # 1. BOX GROUPING BERDASARKAN KELAS
                for i in range(det_count):
                    class_id = int(results[0].boxes.cls[i].cpu().numpy())
                    class_name = self.model.names[class_id]
                    box = results[0].boxes.xyxy[i].cpu().numpy() # [x1, y1, x2, y2]

                    if class_name not in grouped_boxes:
                        grouped_boxes[class_name] = []
                    grouped_boxes[class_name].append(box)
                
                # 2. Jika ada tiang gate yang terdeteksi (bisa 1 tiang atau 2 tiang)
                for class_name, boxes in grouped_boxes.items():
                    #cari threshold untuk menggabungkan box (misal 50% overlap)
                    min_x1 = min(box[0] for box in boxes)
                    min_y1 = min(box[1] for box in boxes)
                    max_x2 = max(box[2] for box in boxes)
                    max_y2 = max(box[3] for box in boxes)

                    # 3. Hitung Titik Tengah Sejati dari Kotak Gabungan
                    center_x = (min_x1 + max_x2) / 2.0
                    center_y = (min_y1 + max_y2) / 2.0
                    area_z = (max_x2 - min_x1) * (max_y2 - min_y1)

                    # 4. Normalisasi Koordinat (-1 sampai 1) sesuai kode awal
                    normalized_x = (center_x - img_width / 2.0) / (img_width / 2.0)
                    normalized_y = (center_y - img_height / 2.0) / (img_height / 2.0)
                    normalized_z = area_z / (img_width * img_height) if (img_width * img_height) > 0 else 0.0
                    
                    all_objects_data.append({
                        'class': class_name,
                        "x": round(float(normalized_x), 3),
                        "y": round(float(normalized_y), 3),
                        "z": round(float(normalized_z), 3)
                    })

                    #Viz di openCV
                    cv2.circle(annotated_frame, (int(center_x), int(center_y)), 6, (0, 0, 255), -1)


            #json msgs
            json_msg = String()
            json_msg.data = json.dumps(all_objects_data)
            self.coord_publisher_.publish(json_msg)
                
            # Publish gambar akhir ke RQT
            ros_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.image_publisher_.publish(ros_msg)

        except Exception as e:
            self.get_logger().error(f'Processing Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
