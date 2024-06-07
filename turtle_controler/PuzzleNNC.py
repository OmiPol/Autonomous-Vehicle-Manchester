import rclpy
from ultralytics import YOLO
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from yolo_msgs.msg import InferenceResult
from yolo_msgs.msg import Yolov8Inference

from std_msgs.msg import String, Float32, Int32

bridge = CvBridge()

class CameraSubscrber(Node):
    def __init__(self):
        super().__init__("yolo_cnn_object_detector")
        self.get_logger().info("Yolo Subscriber Started...!!!")
        self.img_pub = self.create_publisher(Image, '/image_inference_result', 1)
        self.igm_sub = self.create_subscription(Image, '/video_source/raw',self.camera_callback,10)
        self.model = YOLO('/home/ivndx/ros2_ws/src/turtle_controler/turtle_controler/Test5.pt')
        self.yolov8_inference = Yolov8Inference()
        self.yolov8_pub = self.create_publisher(Yolov8Inference, '/yolov8_inference', 1)
        self.minimal_valid_confidence = 0.5


    def camera_callback(self, data):
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        result = self.model(img, verbose=False, conf=self.minimal_valid_confidence)
        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        signal_detected = None

        boxes = result[0].boxes
        for box in boxes:
            bounding_box = box.xyxy[0]
            name_class_detected = self.model.names[int(box.cls)]
            confidence = float(box.conf[0])
            #print(name_class_detected + " detected with " + str(confidence) + " of confidence")

            self.inference_result = InferenceResult()
            self.inference_result.class_name = name_class_detected
            self.inference_result.confidence = confidence
            self.inference_result.top = int(bounding_box[0])
            self.inference_result.left = int(bounding_box[1])
            self.inference_result.bottom = int(bounding_box[2])
            self.inference_result.right = int(bounding_box[3])
            self.yolov8_inference.yolov8_inference.append(self.inference_result)

<<<<<<< Updated upstream
            if name_class_detected != signal_detected:
                if name_class_detected == "turn_left_ahead" or name_class_detected == "turn_right_ahead" or name_class_detected == "ahead_only":
                    signal_detected = name_class_detected
                    self.pub_turn_sign.publish(String(data=name_class_detected))
            else:
                pass
            
            if name_class_detected == "roadwork_ahead" or name_class_detected == "give_way" or name_class_detected == "stop":
                signal_detected = name_class_detected
                self.pub_warn_sign.publish(String(data=name_class_detected))
            else:
                pass
=======
            #self.pub_sign.publish(String(data=name_class_detected))
>>>>>>> Stashed changes

        annotated_frame = result[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')

        self.img_pub.publish(img_msg)
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

def main(args=None):
    rclpy.init(args=args)
    nodeh = CameraSubscrber()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("Node morido >:( ")

if __name__ == "__main__":
    main()
            