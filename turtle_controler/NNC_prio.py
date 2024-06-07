import rclpy
from ultralytics import YOLO
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from yolo_msgs.msg import InferenceResult
from yolo_msgs.msg import Yolov8Inference

from std_msgs.msg import String, Float32, Int32

class NNC_Subscrber(Node):
    def __init__(self):
        super().__init__("yolo_cnn_object_detector")
        self.get_logger().info("Yolo Subscriber Started...!!!")
        self.nnc_sub = self.create_subscription(Yolov8Inference, '/yolov8_inference', self.prio_callback,10)
        self.pub_turn_sign = self.create_publisher(String, '/turn_sign', 1)
        self.pub_warn_sign = self.create_publisher(String, '/warn_sign', 1)
        self.pub_sema_sign = self.create_publisher(String, '/streetlight', 1)
        self.minimal_valid_confidence = 0.5
        self.yolov8 = None


    def prio_callback(self, data): 
        priority_turn = 0
        priority_warn = 0
        priority_sema = 0

        for detected in data.yolov8_inference:
            print("holiwis")
            area = (detected.right - detected.left) * (detected.bottom - detected.top)            
            if(detected.class_name == "ahead_only" or detected.class_name == "turn_right_ahead" or detected.class_name == "turn_left_ahead") and (detected.confidence > 0.9):
                if(detected.class_name == "ahead_only") and (area >= 3200):
                    priority_turn = 2
                    self.pub_turn_sign.publish(String(data=detected.class_name))
                elif(detected.class_name == "turn_right_ahead") and (priority_turn < 2) and (area >= 3200):
                    priority_turn = 1
                    self.pub_turn_sign.publish(String(data=detected.class_name))
                elif(detected.class_name == "turn_left_ahead") and (priority_turn < 2) and (area >= 3200):
                    priority_turn = 0
                    self.pub_turn_sign.publish(String(data=detected.class_name))

                else:
                    pass
                
            elif(detected.class_name == "stop" or detected.class_name == "give_way" or detected.class_name == "roadwork_ahead") and (detected.confidence > 0.9):
                if(detected.class_name == "roadwork_ahead") and (area >= 3200):
                    priority_warn = 2
                    self.pub_warn_sign.publish(String(data=detected.class_name))
                elif(detected.class_name == "give_way") and (priority_warn < 2) and (area >= 3200):
                    priority_warn = 1
                    self.pub_warn_sign.publish(String(data=detected.class_name))
                elif(detected.class_name == "stop") and (priority_warn < 2) and (area >= 3200):
                    priority_warn = 0
                    self.pub_warn_sign.publish(String(data=detected.class_name))
                else:
                    pass
                
            elif(detected.class_name == "red" or detected.class_name == "yellow" or detected.class_name == "green") and (detected.confidence > 0.9):
                if(detected.class_name == "red") and (area >= 520):
                    priority_sema = 2
                    self.pub_sema_sign.publish(String(data=detected.class_name))
                elif(detected.class_name == "yellow") and (priority_sema < 2) and (area >= 520):
                    priority_sema = 1
                    self.pub_sema_sign.publish(String(data=detected.class_name))
                elif(detected.class_name == "green") and (priority_sema < 2) and (area >= 520):
                    priority_sema = 0
                    self.pub_sema_sign.publish(String(data=detected.class_name))
                else:
                    pass

def main(args=None):
    rclpy.init(args=args)
    nodeh = NNC_Subscrber()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("Node morido >:( ")

if __name__ == "__main__":
    main()
            