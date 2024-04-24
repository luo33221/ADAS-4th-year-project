import cv2
import csv
from realsense_camera import RealsenseCamera
from rcnn import RCNN
import os

rs = RealsenseCamera()
intrinsics = rs.get_intrinsics()
mrcnn = RCNN("person_info.csv", intrinsics)
csv_filename = "person_info.csv"

if not os.path.exists(csv_filename): #Checking and Creating CSV File
    with open(csv_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["FrameIndex", "ClassName", "X1", "Y1", "X2", "Y2", "CenterX", "CenterY", "Distance", "World Coordinates"])
frame_index = 0

while True: #Captures frames from the camera, retrieves bounding boxes, classes, and centers of detected objects
    ret, bgr_frame, depth_frame = rs.get_frame_stream()

    boxes, classes, centers = mrcnn.detect_objects(bgr_frame)

    person_boxes = [] #Filters out only person
    person_classes = []
    person_centers = []
    for box, class_id, center in zip(boxes, classes, centers):
        if mrcnn.classes[int(class_id)] == "person":
            person_boxes.append(box)
            person_classes.append(class_id)
            person_centers.append(center)

    mrcnn.draw_object_info(bgr_frame, depth_frame)

    if person_boxes: #If a person is detected, it will record the information into the CSV file.
        mrcnn.record_object_info(frame_index, bgr_frame, depth_frame, person_boxes, person_classes, person_centers)

    cv2.imshow("Bgr frame", bgr_frame)

    key = cv2.waitKey(1)
    if key == 27:
        break

    frame_index += 1

rs.release()
cv2.destroyAllWindows()
