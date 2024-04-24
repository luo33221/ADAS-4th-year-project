import pyrealsense2 as rs
import numpy as np

class RealsenseCamera:
    def __init__(self):
        print("Loading")
        self.pipeline = rs.pipeline() #Initialize the RealSense camera

        config = rs.config() #Configure the resolution and frame
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

        self.pipeline.start(config)
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        profile = self.pipeline.get_active_profile()
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        self.intrinsics = color_profile.get_intrinsics() #Retrieve the camera intrinsics

    def get_frame_stream(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            print("Error")
            return False, None, None

        spatial = rs.spatial_filter()
        spatial.set_option(rs.option.holes_fill, 3)
        filtered_depth = spatial.process(depth_frame)

        hole_filling = rs.hole_filling_filter() #Fill in missing depth values in depth frames
        filled_depth = hole_filling.process(filtered_depth)

        depth_image = np.asanyarray(filled_depth.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return True, color_image, depth_image
    
    def get_intrinsics(self):
        return self.intrinsics

    def release(self):
        self.pipeline.stop()