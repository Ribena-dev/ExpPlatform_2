import pyrealsense2 as rs
import cv2
import time
import numpy as np

camera_startup_pause = 2
output_filename = "test1"
fps = 30
video_length = 30
camera_pair = "Intel Lidar L515"

#Turn on emitting camera
# pipeline_emit = rs.pipeline()
# config_emit = rs.config()
# pipeline_emit.start(config_emit)

#Fire up recording camera
pipeline_record = rs.pipeline()
config_record = rs.config()

#Setup streaming and recording
config_record.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, fps)
config_record.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, fps)


bag_filename = output_filename + ".bag"    
config_record.enable_record_to_file(bag_filename)

#Turn on recording pipeline
pipeline_record_profile = pipeline_record.start(config_record)
device_record = pipeline_record_profile.get_device()
device_recorder = device_record.as_recorder()

#getting the depth sensor profile
depth_sensor = device_record.first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

#Set autoexposure priority to False to improve frame rate acquisition
color_sensor_record = device_record.query_sensors()[1]
color_sensor_record.set_option(rs.option.auto_exposure_priority, False)

#Start recording!
try:
    #Pause before recording to let camera warm up
    rs.recorder.pause(device_recorder)
    print('Pausing.')
    
    for t in range(camera_startup_pause*2):
        print('\r','Pausing','.'*(t+2))
        time.sleep(0.5)

    print()    
    rs.recorder.resume(device_recorder)
    
    #For colorizing the depth frame
    colorizer = rs.colorizer()

    start=time.time()
    while time.time() - start < int(video_length):
        frames = pipeline_record.wait_for_frames()
        frames.keep() #Reduces frame drops (I think!)
        
        #Get color and depth frames for display
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        
        color_frame = np.asanyarray(color_frame.get_data())
        color_frame = cv2.cvtColor(color_frame, cv2.COLOR_RGB2BGR)

        depth_image = np.asanyarray(depth_frame.get_data())
        depth_frame = np.asanyarray(colorizer.colorize(depth_frame).get_data())
        
        depth = depth_image[240][320].astype(float) # resolution is 640 x 480 ; define row first, then column
        distance = depth * depth_scale
        print("Distance is: ", distance) 
          
        #Stack color and depth frames vertically for display
        camera_images = np.vstack((color_frame, depth_frame))
        
        #Display images
        cv2.namedWindow('Camera ' + str(camera_pair), cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Camera ' + str(camera_pair), camera_images)
        # cv2.setMouseCallback('Camera ' + str(camera_pair, color_to_depth))
        cv2.waitKey(5)
        
finally:
    pipeline_record.stop()
    cv2.destroyAllWindows()

# value_pairs = []
# def color_to_depth(event, x, y, flags, param):
#     global z
#     if event = cv2.EVENT_LBUTTONDOWN:
#         value_pairs.append(z, disparity[y,x])

# def 
# depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
# depth_min = 0.11 #meter
# depth_max = 1.0 #meter

# depth_intrin = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
# color_intrin = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

# depth_to_color_extrin =  profile.get_stream(rs.stream.depth).as_video_stream_profile().get_extrinsics_to( profile.get_stream(rs.stream.color))
# color_to_depth_extrin =  profile.get_stream(rs.stream.color).as_video_stream_profile().get_extrinsics_to( profile.get_stream(rs.stream.depth))

# color_points = [
#     [400.0, 150.0],
#     [560.0, 150.0],
#     [560.0, 260.0],
#     [400.0, 260.0]
# ]
# for color_point in color_points:
#    depth_point_ = rs.rs2_project_color_pixel_to_depth_pixel(
#                 depth_frame.get_data(), depth_scale,
#                 depth_min, depth_max,
#                 depth_intrin, color_intrin, depth_to_color_extrin, color_to_depth_extrin, color_point)
