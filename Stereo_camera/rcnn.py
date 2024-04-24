import cv2
import numpy as np
import csv
import pyrealsense2 as rs

class RCNN:
    def __init__(self, csv_filename, intr):
        
        self.intr = intr #Camera intrinsic parameters
        self.net = cv2.dnn.readNetFromTensorflow("dnn/frozen_inference_graph_coco.pb",
                                                "dnn/mask_rcnn_inception_v2_coco_2018_01_28.pbtxt")#Load the pre-trained Mask R-CNN model
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

        np.random.seed(2)
        self.colors = np.random.randint(0, 255, (90, 3))

        self.detection_threshold = 0.7 #Set the detection threshold

        self.classes = []
        with open("dnn/classes.txt", "r") as file_object: #Load the classname from txt file, map class IDs to human-readable names
            for class_name in file_object.readlines():
                class_name = class_name.strip()
                self.classes.append(class_name)

        self.obj_boxes = []
        self.obj_classes = []
        self.obj_centers = []
        self.distances = []
        self.csv_filename = csv_filename

    def detect_objects(self, bgr_frame):
        blob = cv2.dnn.blobFromImage(bgr_frame, swapRB=True) #Converts the BGR frame to a blob
        self.net.setInput(blob) #Input the blob to the RCNN model
        boxes = self.net.forward("detection_out_final")
        frame_height, frame_width, _ = bgr_frame.shape #Retrieve the height and width
        detection_count = boxes.shape[2] #Calculate the number of detected objects.

        self.obj_boxes = []
        self.obj_classes = []
        self.obj_centers = []

        for i in range(detection_count):
            box = boxes[0, 0, i]
            class_id = box[1]
            score = box[2]

            if score < self.detection_threshold: #Check the confidence score of the detection
                continue # Continue if it's greater than the detection threshold.

            x = int(box[3] * frame_width) #Calculate the pixel coordinates of the bounding box
            y = int(box[4] * frame_height)
            x2 = int(box[5] * frame_width)
            y2 = int(box[6] * frame_height)
            self.obj_boxes.append([x, y, x2, y2])

            cx = (x + x2) // 2 #Calculate the center coordinates of the bounding box
            cy = (y + y2) // 2
            self.obj_centers.append((cx, cy))

            self.obj_classes.append(class_id)

        return self.obj_boxes, self.obj_classes, self.obj_centers

    def draw_object_info(self, bgr_frame, depth_frame): #draw information of the objects
        for box, class_id, obj_center in zip(self.obj_boxes, self.obj_classes, self.obj_centers):
            x, y, x2, y2 = box

            color = self.colors[int(class_id)]
            color = (int(color[0]), int(color[1]), int(color[2]))

            cx, cy = obj_center

            depth_mm = depth_frame[cy, cx] #Calculate the depth of each detected object using the center point depth value

            cv2.line(bgr_frame, (cx, y), (cx, y2), color, 1)
            cv2.line(bgr_frame, (x, cy), (x2, cy), color, 1)

            class_name = self.classes[int(class_id)]

            cv2.rectangle(bgr_frame, (x, y), (x2, y2), color, 1)

            cv2.putText(bgr_frame, class_name.capitalize(), (x + 5, y + 25), 0, 0.8, (255, 255, 255), 2) #print the text on the detection window
            cv2.putText(bgr_frame, "{} cm".format(depth_mm / 10), (x + 5, y + 60), 0, 1.0, (255, 255, 255), 2)

            world_coordinates = self.get_3d_position(cx, cy, depth_mm)
            #Calculate the 3D position (world coordinates) of a point in space from pixel coordinates and depth value
            print(f"Object {class_name} at pixel position ({cx}, {cy}) has 3D coordinates: {world_coordinates}")

    def record_object_info(self, frame_index, bgr_frame, depth_frame, boxes, classes, centers): #Record informations in a CSV file
     with open(self.csv_filename, mode='a', newline='') as file:
        writer = csv.writer(file)

        for box, class_id, obj_center in zip(boxes, classes, centers):
            x, y, x2, y2 = box

            cx, cy = obj_center

            depth_mm = depth_frame[cy, cx]

            class_name = self.classes[int(class_id)]
            distance_cm = depth_mm / 10

            world_coordinates = self.get_3d_position(cx, cy, depth_mm)
            
            writer.writerow([frame_index, class_name, x, y, x2, y2, cx, cy, distance_cm, world_coordinates])

     return bgr_frame


    def get_3d_position(self, pixel_x, pixel_y, depth_mm):
        #Uses the RealSense SDK to convert pixel coordinates and depth value to world coordinates
        depth_point = rs.rs2_deproject_pixel_to_point(self.intr, [pixel_x, pixel_y], depth_mm)
        return depth_point