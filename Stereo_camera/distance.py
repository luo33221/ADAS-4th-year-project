import pyrealsense2 as rs
import numpy as np
import cv2
import time
import csv

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

pipeline.start(config)

selected_pixel = None
last_print_time = 0
print_interval = 0.01

csv_filename = "depth_info.csv"

def on_mouse_move(event, x, y, flags, param):
    global selected_pixel
    selected_pixel = (x, y)

cv2.namedWindow("Color Image")
cv2.setMouseCallback("Color Image", on_mouse_move)

# Open CSV file in write mode
with open(csv_filename, mode='w', newline='') as csv_file:
    fieldnames = ['Timestamp', 'X', 'Y', 'Depth (mm)']
    writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    writer.writeheader()

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:

                continue

            color_image = np.asanyarray(color_frame.get_data())

            cv2.imshow("Color Image", color_image)

            if selected_pixel:
                current_time = time.time()
                if current_time - last_print_time >= print_interval:
                    x, y = selected_pixel
                    depth_mm = depth_frame.get_distance(x, y) * 1000  # Convert to millimeters
                    print(f"Depth at pixel ({x}, {y}): {depth_mm} mm")

                    # Write to CSV file
                    writer.writerow({
                        'Timestamp': current_time,
                        'X': x,
                        'Y': y,
                        'Depth (mm)': depth_mm
                    })

                    last_print_time = current_time

            if cv2.waitKey(1) & 0xFF == 27:
                break

    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()

cv2.destroyAllWindows()
