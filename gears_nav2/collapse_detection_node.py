import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from sensor_msgs.msg import Image  # Image is the message type
import numpy as np  # Import Numpy library
from collections import defaultdict
import cv2
import numpy as np
import torch
from ultralytics import YOLO
from collapse_detection.model.collapse_detection_model.attentionBiLSTM import (
    ActionRecognizationModel,
)
import logging
import pygame
from foxglove_msgs.msg import (
    ImageAnnotations,
    TextAnnotation,
)

# Replace 'ultralytics' with the correct logger name if different.

ZED_HEIGHT = 360
ZED_WIDTH = 640


class CollapseDectionNode(Node):
    """
    Create an CollapseDectionNode class, which is a subclass of the Node class.
    """

    def __init__(self):

        super().__init__("collapse_detection_node")

        self.subscription = self.create_subscription(
            Image, "/zed/zed_node/rgb/image_rect_color", self.rgb_image_callback, 10
        )
        self.bridge = CvBridge()
        timer_period = 1.0 / 10  # seconds (20Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.current_frame = None

        model_weight_path = "/home/minegearscsu/ros2_ws/src/gears_nav2/collapse_detection/collapse_detection_model0.pt"
        self.yolo_model = YOLO("yolov8n-pose.pt")
        self.action_recognition_model = ActionRecognizationModel(
            input_dim=6, output_dim=6, hidden_dim=128, num_layers=128
        )
        self.track_history = defaultdict(lambda: [])
        self.sequence = 30
        self.model_weight = torch.load(model_weight_path)

        logging.getLogger("ultralytics").setLevel(logging.WARNING)

        pygame.mixer.init()
        pygame.mixer.music.load(
            "/home/minegearscsu/ros2_ws/src/gears_nav2/resource/Collapse Detected.mp3"
        )

        self.collapse_detected = False

        self.collapse_detected_publisher = self.create_publisher(
            ImageAnnotations, "image_annotations", 10
        )
        self.person_x = 0.0
        self.person_y = 0.0

    def timer_callback(self):

        if self.collapse_detected:
            # self.get_logger().info(f"PER X {self.person_x} PER Y {self.person_y}")
            msg = ImageAnnotations()
            text = TextAnnotation()
            text.position.x = self.person_x
            text.position.y = self.person_y
            text.text = "Collapse Detected"
            text.font_size = 30.0
            text.text_color.r = 1.0
            text.text_color.g = 1.0
            text.text_color.b = 1.0
            text.text_color.a = 1.0

            text.background_color.r = 1.0
            text.background_color.g = 0.0
            text.background_color.b = 0.0
            text.background_color.a = 0.5
            msg.texts.append(text)
            # self.collapse_detected_publisher.publish(msg)
            # self.get_logger().info("Publishing ImageAnnotations")
        else:
            msg = ImageAnnotations()
        self.collapse_detected_publisher.publish(msg)

    def rgb_image_callback(self, data):
        # cv2.namedWindow("YOLOv8 Tracking", cv2.WINDOW_NORMAL)
        # cv2.resizeWindow("YOLOv8 Tracking", ZED_WIDTH, ZED_HEIGHT)

        # self.get_logger().info(f"ID {(self.current_frame)}")
        self.action_recognition_model.load_state_dict(self.model_weight)
        self.action_recognition_model.eval()

        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            results = self.yolo_model.track(self.current_frame, persist=True)

            if results[0].boxes.id is not None:
                # Get the boxes and track IDs
                boxes = results[0].boxes.xywh.cpu()
                normalize_keypoints = results[0].keypoints.xyn.cpu()
                normalize_boxes_xywh = results[0].boxes.xywhn.cpu()
                normalize_boxes_xyxy = results[0].boxes.xyxyn.cpu()
                track_ids = results[0].boxes.id.int().cpu().tolist()
                # Visualize the results on the frame
                annotated_frame = results[0].plot()

                # Iterate over the visible IDs in the frame
                for track_id in list(self.track_history.keys()):
                    if track_id not in track_ids:
                        # If the ID is not visible in the frame, pop it from the history
                        self.track_history.pop(track_id, None)
                    else:
                        if len(self.track_history[track_id]) == self.sequence:
                            input = torch.stack(self.track_history[track_id])
                            input = torch.unsqueeze(input, dim=0)

                            output = self.feed_to_model(input)

                            # Get the coordinates for putting text on the frame
                            x, y, w, h = boxes[track_ids.index(track_id)]
                            text_position = (
                                int(x),
                                int(y - 10),
                            )  # Adjust position based on your requirement
                            self.person_x = float(x)
                            self.person_y = float(y - 20)
                            # self.get_logger().info(f"Output: {type(output)}")
                            if output == 1:
                                # self.get_logger().info(f"COLLAPSE DETECTED!!---------")
                                # cv2.putText(
                                #     annotated_frame,
                                #     "COLLAPSE DETECTED!",
                                #     text_position,
                                #     cv2.FONT_HERSHEY_SIMPLEX,
                                #     2,
                                #     (0, 180, 255),
                                #     7,
                                # )
                                self.collapse_detected = True

                                if not pygame.mixer.music.get_busy():
                                    pygame.mixer.music.play()
                            else:
                                # self.get_logger().info(
                                #     f"NO ACCIDENT!!- NO COLLAPSE DETECTED!"
                                # )
                                # cv2.putText(
                                #     annotated_frame,
                                #     "SAFE",
                                #     text_position,
                                #     cv2.FONT_HERSHEY_SIMPLEX,
                                #     2,
                                #     (0, 255, 0),
                                #     7,
                                # )
                                self.collapse_detected = False
                        # else:
                        #     print(
                        #         f"ID: {track_id} ===================== { len(self.track_history[track_id]) }"
                        #     )
                # Plot the tracks
                for box, track_id, n_keypoint, n_boxes_xywh, n_boxes_xyxy in zip(
                    boxes,
                    track_ids,
                    normalize_keypoints,
                    normalize_boxes_xywh,
                    normalize_boxes_xyxy,
                ):
                    x, y, w, h = box
                    track = self.track_history[track_id]

                    relative_keypoints = self.extract_normalize_keypoints(
                        n_boxes_xywh, n_boxes_xyxy, n_keypoint
                    )

                    track.append(relative_keypoints)  # store keypoints

                    while len(track) > self.sequence:  # retain 30 tracks for 30 frames
                        track.pop(0)

                # Display the annotated frame
            #     cv2.imshow("YOLOv8 Tracking", annotated_frame)
            #     if cv2.waitKey(1) & 0xFF == ord("q"):
            #         return
            # else:

            #     cv2.imshow("YOLOv8 Tracking", self.current_frame)
            #     if cv2.waitKey(1) & 0xFF == ord("q"):
            #         return
            # pass
            else:
                self.collapse_detected = False
        except:
            pass

    def extract_normalize_keypoints(
        self, normalize_bbox_xywh, normalize_bbox_xyxy, normalize_keypoints
    ):
        relative = normalize_keypoints - normalize_bbox_xyxy[:2]
        relative = relative / normalize_bbox_xywh[2:]

        return relative

    def feed_to_model(self, input):
        output = self.action_recognition_model(input.to("cuda:0"))
        predicted_labels = (torch.sigmoid(output) > 0.5).float()

        return predicted_labels


def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    collapse_detection_node = CollapseDectionNode()

    # Spin the node so the callback function is called.
    rclpy.spin(collapse_detection_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    collapse_detection_node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()
