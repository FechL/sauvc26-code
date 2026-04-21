import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point # <--- 1. IMPORT POINT MESSAGE
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
        self.model = YOLO("best.pt")
        # self.model = YOLO("final_test.pt")
        self.get_logger().info("YOLO Node Initialized. Publishing Image and Coordinates.")

    def listener_callback(self, msg):
        try:
            self.get_logger().info("Received image, processing...")
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Run Inference
            results = self.model(cv_image, verbose=False, conf=0.1)
            annotated_frame = results[0].plot()

            det_count = len(results[0].boxes)
            self.get_logger().info(f"Detected {det_count} objects")
            if det_count > 0:
                
                x_sum = 0.0
                y_sum = 0.0
                z_sum = 0.0
                x_list = []
                y_list = []
                z_list = []
                img_height, img_width = cv_image.shape[:2]

                for i in range(det_count):
                    class_id = int(results[0].boxes.cls[i].cpu().numpy())
                    class_name = self.model.names[class_id]

                    if class_name != "gate":
                        continue
                    
                    add = True
                    box = results[0].boxes.xyxy[i].cpu().numpy()
                    
                    x1, y1, x2, y2 = box
                    x = (x1 + x2) / 2.0
                    y = (y1 + y2) / 2.0
                    z = (x2 - x1) * (y2 - y1)


                    if i > 0:
                        for j in range(len(x_list)):
                            # skip box dengan koordinat terdekat
                            if abs(x - x_list[j]) < img_width*(3/100) and abs(y - y_list[j]) < img_height*(3/100):
                                add = False
                                break
                
                    if add:
                        x_list.append(x)
                        y_list.append(y)
                        z_list.append(z)
            
                len_list = len(x_list)

                if len_list > 1:

                    x_sum += sum(x_list)
                    y_sum += sum(y_list)
                    z_sum += sum(z_list)
                    
                    # calculate center points
                    center_x = x_sum / len_list
                    center_y = y_sum / len_list
                    center_z = z_sum / len_list
                    
                    # publish coordinates
                    coord_msg = Point()
                    
                    normalized_x = (center_x - img_width / 2.0) / (img_width / 2.0) # normalize coordinates to -1 to 1 range
                    coord_msg.x = float(normalized_x)
                    normalized_y = (center_y - img_height / 2.0) / (img_height / 2.0)
                    coord_msg.y = float(normalized_y)
                    if (img_width * img_height) > 0:
                        normalized_z = (center_z - 0) / (img_width * img_height - 0)  # Normalize z to 0-1 range
                        coord_msg.z = float(normalized_z)

                    self.coord_publisher_.publish(coord_msg)
                    self.get_logger().info(f"Published normalized coords: x={normalized_x:.3f}, y={normalized_y:.3f}, z={normalized_z:.3f}")
                    
                    # Draw a red dot at the center so you can see it in RQT
                    cv2.circle(annotated_frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)

            # Publish the annotated image
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
