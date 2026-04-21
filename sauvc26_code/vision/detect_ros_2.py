import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
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
        self.coord_publisher_ = self.create_publisher(Point, '/yolo_target_coord', 10)

        self.bridge = CvBridge()
        self.model = YOLO("final_test.pt")
        self.get_logger().info("YOLO Node Initialized. Publishing Image and Coordinates.")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            img_height, img_width = cv_image.shape[:2]
            
            # Run Inference
            results = self.model(cv_image, verbose=False, conf=0.1)
            annotated_frame = results[0].plot()

            det_count = len(results[0].boxes)
            
            if det_count > 0:
                gate_boxes = []

                # 1. Kumpulkan semua koordinat deteksi yang bernama "gate" (tiang-tiangnya)
                for i in range(det_count):
                    class_id = int(results[0].boxes.cls[i].cpu().numpy())
                    class_name = self.model.names[class_id]

                    if class_name == "gate":
                        box = results[0].boxes.xyxy[i].cpu().numpy()
                        gate_boxes.append(box) # box berisi [x1, y1, x2, y2]
                
                # 2. Jika ada tiang gate yang terdeteksi (bisa 1 tiang atau 2 tiang)
                if len(gate_boxes) > 0:
                    # Cari batas paling ujung dari semua tiang untuk membuat 1 Kotak Gabungan
                    x1_list = [b[0] for b in gate_boxes]
                    y1_list = [b[1] for b in gate_boxes]
                    x2_list = [b[2] for b in gate_boxes]
                    y2_list = [b[3] for b in gate_boxes]

                    min_x1 = min(x1_list) # Ujung paling kiri
                    min_y1 = min(y1_list) # Ujung paling atas
                    max_x2 = max(x2_list) # Ujung paling kanan
                    max_y2 = max(y2_list) # Ujung paling bawah

                    # 3. Hitung Titik Tengah Sejati dari Kotak Gabungan
                    center_x = (min_x1 + max_x2) / 2.0
                    center_y = (min_y1 + max_y2) / 2.0
                    area_z = (max_x2 - min_x1) * (max_y2 - min_y1)

                    # 4. Normalisasi Koordinat (-1 sampai 1) sesuai kode awal
                    normalized_x = (center_x - img_width / 2.0) / (img_width / 2.0)
                    normalized_y = (center_y - img_height / 2.0) / (img_height / 2.0)
                    normalized_z = 0.0
                    if (img_width * img_height) > 0:
                        normalized_z = area_z / (img_width * img_height)

                    # 5. Publish Data
                    coord_msg = Point()
                    coord_msg.x = float(normalized_x)
                    coord_msg.y = float(normalized_y)
                    coord_msg.z = float(normalized_z)
                    self.coord_publisher_.publish(coord_msg)

                    # --- VISUALISASI TAMBAHAN ---
                    # Gambar kotak gabungan warna Hijau agar terlihat bahwa 2 tiang sudah menyatu
                    cv2.rectangle(annotated_frame, (int(min_x1), int(min_y1)), (int(max_x2), int(max_y2)), (0, 255, 0), 3)
                    # Gambar titik merah di tengah-tengahnya
                    cv2.circle(annotated_frame, (int(center_x), int(center_y)), 6, (0, 0, 255), -1)

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
