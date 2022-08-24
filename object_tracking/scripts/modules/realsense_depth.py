#-*-utf-8-*-
'''
Class file for realsense wrapper
'''
import pyrealsense2 as rs
import numpy as np

#seral number corresponding to the used cameras
#if different cammeras are used they probably need to be changed
D415_SERIAL = '012322061385'
D435_SERIAL = '109122071122'
class DepthCamera:
    def __init__(self, width, height, cam):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()

        self.cam = cam
        if (self.cam == 'd435'):
            config.enable_device(D435_SERIAL)
        else:
            config.enable_device(D415_SERIAL)



        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)
        if (self.cam == 'd435'):
            config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 63)
            config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)




        # Start streaming
        self.profile = self.pipeline.start(config)
    def motion_to_np(self, data):
        '''
        changes axis of data to axis the robot expects 
        '''
        return np.asarray([data.z, -data.x, -data.y])

    def get_frame(self):
        '''
        Gets the current image frames and possible accel data

        Returns:
        True: unecessary but exists to avoid breaking other code kind of
        depth_image: numpy array of depth in mm image
        color_image: numpy array of color data for the image
        accel: list of accel data
        gyro: list of gyro data
        '''
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        accel, gyro = None, None
        if (self.cam == 'd435'):
            accel = self.motion_to_np(frames[2].as_motion_frame().get_motion_data())
            gyro = self.motion_to_np(frames[3].as_motion_frame().get_motion_data())

            

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        return True, depth_image, color_image, accel, gyro

    def get_raw_frame(self):
        '''
        gets the realsense objects containing the data
        some things might want some info contained in the class, maybe
        '''
        frames = self.pipeline.wait_for_frames()
        return frames.get_depth_frame(), frames.get_color_frame()
    def get_intrinsics(self):
        '''
        gets the intrinsics related to the camera like the focal length and similar
        '''
        return self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

    def release(self):
        '''theoretically releases the camera from the being used, not sure any actual effect'''
        self.pipeline.stop()
