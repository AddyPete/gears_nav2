from collections import defaultdict
import cv2
import numpy as np
import torch 
from ultralytics import YOLO
from model.collapse_detection_model.attentionBiLSTM import ActionRecognizationModel

class CollapseDetectionApplication:
    def __init__(self, source, window_width, window_height, model_weight_path) -> None:
        print(model_weight_path)
        self.yolo_model = YOLO('yolov8n-pose.pt')
        self.action_recognition_model = ActionRecognizationModel(input_dim = 6, output_dim = 6, hidden_dim=128, num_layers=128)
        self.source = source
        self.track_history = defaultdict(lambda: [])
        self.sequence = 30
        self.window_width = window_width
        self.window_height = window_height
        self.model_weight = torch.load(model_weight_path)
    

    def extract_normalize_keypoints(self, normalize_bbox_xywh, normalize_bbox_xyxy, normalize_keypoints):
        relative = normalize_keypoints - normalize_bbox_xyxy[:2]
        relative = relative / normalize_bbox_xywh[2:]

        return relative
    
    def feed_to_model(self, input):
        output = self.action_recognition_model(input.to("cuda:0"))
        predicted_labels = (torch.sigmoid(output) > 0.5).float()
        
        return predicted_labels
    
    def runs(self):
        cv2.namedWindow("YOLOv8 Tracking", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("YOLOv8 Tracking", self.window_width, self.window_height)
        self.action_recognition_model.load_state_dict(self.model_weight)
        self.action_recognition_model.eval()

        cap = cv2.VideoCapture(self.source)

        while cap.isOpened():
            success, frame = cap.read()
            if success:
                # Run YOLOv8 tracking on the frame, persisting tracks between frames
                results = self.yolo_model.track(frame, persist=True)


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
                                text_position = (int(x), int(y - 10))  # Adjust position based on your requirement

                                if output == 1:
                                    print("COLLAPSE DETECTED!!---------")
                                    cv2.putText(annotated_frame, "COLLAPSE DETECTED!", text_position, cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 180, 255), 7)
                                else:
                                    print("NO ACCIDENT!!- NO COLLAPSE DETECTED!")
                                    cv2.putText(annotated_frame, "SAFE", text_position, cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 7)
                            else: 
                                print(f"ID: {track_id} ===================== { len(self.track_history[track_id]) }")
                    # Plot the tracks
                    for box, track_id, n_keypoint, n_boxes_xywh, n_boxes_xyxy in zip(boxes, track_ids, normalize_keypoints, normalize_boxes_xywh, normalize_boxes_xyxy):
                        x, y, w, h = box
                        track = self.track_history[track_id]

                        relative_keypoints = self.extract_normalize_keypoints(n_boxes_xywh, n_boxes_xyxy, n_keypoint)

                        track.append(relative_keypoints)  #store keypoints

                        while len(track) > self.sequence:  # retain 30 tracks for 30 frames
                            track.pop(0)

                    # Display the annotated frame
                    cv2.imshow("YOLOv8 Tracking", annotated_frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                else:
                    cv2.imshow("YOLOv8 Tracking", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
            else:
                # Break the loop if the end of the video is reached
                break 

