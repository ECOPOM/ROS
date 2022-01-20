#https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/align-depth2color.py

import cv2                              # state of the art computer vision algorithms library
import numpy as np                        # fundamental package for scientific computing          
import pyrealsense2 as rs 
import pandas as pd

###### STARTING  AND SET UP THE CAMERA  #############
####################################################
# Create a pipeline
pipeline = rs.pipeline()

#Create a config and configure the pipeline to stream
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 15)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 15)
# -- if needed start also IR cameras and IMU data stream
# config.enable_stream(rs.stream.infrared, 1280, 720, rs.format.y8, 30) 
# config.enable_stream(rs.stream.accel)
# config.enable_stream(rs.stream.gyro)

######## HOW TO SET POST PROCESSING FILTERS###########
# dec_filter = rs.decimation_filter ()   # NON USARE Decimation - reduces depth frame density
spat_filter = rs.spatial_filter()          # Spatial    - edge-preserving spatial smoothing
temp_filter = rs.temporal_filter()    # Temporal   - reduces temporal noise
# ADD HDR MERGE, DE4PTH TO DISPARITY , DISPARITY TO DEPTH
# frames = pipe.wait_for_frames()
# depth_frame = frames.get_depth_frame()
# filtered = dec_filter.process(depth_frame)
# filtered = spat_filter.process(filtered)
# filtered = temp_filter.process(filtered)

#Record the streaming
#config.enable_record_to_file('box_bag') #TO RECORD

# Start streaming
profile = pipeline.start(config)

#SET A PROFILE SENSOR  - (Check how to load a preset_json_file)
depth_sensor = profile.get_device().first_depth_sensor()
depth_sensor.set_option( rs.option.visual_preset, 4)  # Set high density for depth sensor  depth_sensor.set_option(rs.option.visual_preset, Preset.HighAccuracy) #CHECK Other option
#Set Depth Unit
depth_sensor.set_option(rs.option.depth_units, 0.001) # Depth unit:  0.001=1mm; 
#Set Laser On/ Off for structured light - better always ON
set_laser = 150 # values: Range = 0.0-360 -->  0= OFF; 150= Standard-ON;  150-360: Laser improved
depth_sensor.set_option( rs.option.laser_power, set_laser) #Fucntion to set laser power
# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , round(depth_scale, 6), "m")
# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

####### DEFINE ORCHARD + MACHINE VARIABLE  ###########
###########################################
#ORCHARD SET-UP
# We will be removing the background of objects more distant than clipping_distance_in_meters meters away
Interow_distance = float(input("Inter-row distance (in meter): ", ))  #in meter
Max_Canopy_Widht = float(input("Max canopy widht (in meter)",)) #in meter
# canopy_geometry

#clipping distance in field depend in base of the position of the camera:
# 1) OPTION 1 - if placed in tHe middle of the row, then clipping ditance is = to interrow/2 + mAXcanopywidht (e +w SIDFE TOGHETER)
clipping_distance_in_meters =  (Interow_distance/2)+Max_Canopy_Widht  #in meter
clipping_distance = clipping_distance_in_meters / depth_scale
# 2) OPTION 2- if placed on the opposite side of the row is 
# in_row_Canopy_Widht= Max_Canopy_Widht/2
# next_in_row_Canopy_Widht = in_row_Canopy_Widht
# clipping_distance_in_meters =  (Interow_distance-in_row_Canopy_Widht)+next_in_row_Canopy_Widht  #
# clipping_distance = clipping_distance_in_meters / depth_scale
print("Interow_distance: " + str(round(Interow_distance, 3)) + "m")
print("Max_Canopy_Widht: " + str(round(Max_Canopy_Widht, 3)) + "m")
print("Threshold distance: " + str(round(clipping_distance*depth_scale , 3)) + "m")

# MACHINE SET UP
nozzle_num =int(input("how many nuzzle sector are presente in your sprayer?",))
#future parameter
#nozzle_inclination
#nozzle_spraiyng_angle
#nozzle_min_distance
#inter_nozzle_distance

#-------- debug purpose
ROI_Porosity = [] 
TOT_Porosity = []
#----------------------
####### CREATE A ROI FUNCTION  ###########
###########################################
'''
        !!!  DA FINIRE  !!!
def ROI_creator(bg_removed, ROI_numbers):
    img_Height, img_Width, img_Channel = bg_removed.shape
    # if camera is HVertical INVERT : 
    # img_Height, img_Width, img_Channel = bg_removed.shape

    #  GET ROI DIMENSION (a square)
    ROI_Height =img_Height//ROI_numbers
    ROI_Width =ROI_Height

    ROIs_coords_list = []

    starting_pixel =  0
    ending_pixel = 0
        
    starting_pixel_list =  []
    ending_pixel_list = []

    for i in ROI_numbers:
        half_Height = img_Height//2
        starting_pixel =  ((half_Height-ROI_Height//2),0)
        ending_pixel =((starting_pixel[0]+ROI_Height),ROI_Width)
        ROI_coord = (starting_pixel, ending_pixel)
'''

# Streaming loop
try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # ?? add a button to "pause the video"
        if input == "p":
          frames = pipeline()
        else:
          pass
        
        # Align the depth frame to color frame
        aligned_frames = align.process(frames)
        # Get aligned frames 
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()
        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue
                    
        #filtering depth noise
        # NON USARE aligned_depth_frame = dec_filter.process(aligned_depth_frame)
        aligned_depth_frame = spat_filter.process(aligned_depth_frame)
        aligned_depth_frame = temp_filter.process(aligned_depth_frame)
        #aggiungi altri se si cvuole provare -  vedi sopra e agginugi qui
        
        #convert in arrays
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        # Remove background - Set pixels further than clipping_distance to cliping color
        Clipping_color = 0
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), Clipping_color, color_image)

        #Get dimension of the aligned image for processing
        img_Height, img_Width, img_Channel = bg_removed.shape
        
        ############ WHOLE IMAGE POROSITY CALC #################
        Totpixel=bg_removed.size                    # ALL pix in the image
        Tot_Colored_pixel = np.sum(bg_removed > 0)  # pixel closer tha clipping distance
        Tot_B_pixel = np.sum(bg_removed == 0)       # pixel further than clipping distance
        Tot_BW_together = Tot_Colored_pixel+Tot_B_pixel   # Check for the sum of B&W is == Totpixel
        
        Tot_colored_pix_Perc =(Tot_Colored_pixel/Totpixel)*100    #Compute percentege on the Total
        Tot_B_pix_Perc =(Tot_B_pixel/Totpixel)*100    #Compute percentege on the Total
        Tot_BW_tog_Perc =Tot_colored_pix_Perc + Tot_B_pix_Perc 
        
        #Print INfos:
        print('---------- WHOLE FRAME -----------')    
        print('Totpixel SIZE ', Totpixel)    
        print('Tot_Colored_pixel ', Tot_Colored_pixel)
        print('Tot_B_pixel', Tot_B_pixel)
        print('Tot_BW_together - should be == to Totpixel', Tot_BW_together)
        #   print('GRAY_pixel - should be 0 !', GRAY_pixel)
        print('Tot_colored_pix_Perc', Tot_colored_pix_Perc)
        print('Tot_B_pix_Perc', Tot_B_pix_Perc)
        print('Tot_BW_tog_Perc - should be 100%', Tot_BW_tog_Perc)
              
        ####### CREATE ROI/s ###############################
        #ci vorrebbe formula per calcolare in base alla distanza tra telecamera e filare la dimensione della scena voluta(50x50cm?)
        #magari sapendo che area coprono gli spruzzini (da definire sopra)
        
        #FInire ROI_creator function  sopra  
        #     
        #Depth FOV HD = H:87±3 / V:58±1 / D:95±1 
        #IR Projector FOV = H:90 / V:63 / D:99
        #RGB camer FOV = H:69±1 /V:42±1 /D:77±1

        ROI_Y1 =img_Height//6
        ROI_Y2 =ROI_Y1+ ROI_Y1*4
        ROI_X1 =img_Width//3 
        ROI_X2 = ROI_X1 +ROI_X1

        # ROI_Y1 =img_Height//nozzle_num
        # ROI_Y2 =ROI_Y1+ ROI_Y1*nozzle_num
        # ROI_X1 =img_Width//nozzle_num 
        # ROI_X2 = ROI_X1 +ROI_X1
        
        roi =  bg_removed[ROI_X1:ROI_X2,ROI_Y1:ROI_Y2]
        
        ############# ROI IMAGE POROSITY ##############à        
        ROI_Totpixel=roi.size                   # ALL pix in the image
        ROI_Tot_Colored_pixel = np.sum(roi > 0)
        ROI_Tot_B_pixel = np.sum(roi == 0)       # Black pixel after binary threshold
        ROI_Tot_BW_together = ROI_Tot_Colored_pixel+ROI_Tot_B_pixel   # Check for the sum of B&W is == Totpixel
        #GRAY_pixel = np.sum(th1_Image_Gray <255 )  # Check if the binary threshold  worked properly (only 0 e 255 as pixel values)
        ROI_Tot_colored_pix_Perc =(ROI_Tot_Colored_pixel/ROI_Totpixel)*100    #Compute percentege on the Total
        ROI_Tot_B_pix_Perc =(ROI_Tot_B_pixel/ROI_Totpixel)*100    #Compute percentege on the Total
        ROI_Tot_BW_tog_Perc =ROI_Tot_colored_pix_Perc + ROI_Tot_B_pix_Perc 
        
        #Print INfos:
        print('---------- ROI AREA -----------')    
        print('ROI_Totpixel ', ROI_Totpixel)
        print('ROI_Tot_Colored_pixel ', ROI_Tot_Colored_pixel)
        print('ROI_Tot_B_pixel', ROI_Tot_B_pixel)
        print('ROI_Tot_BW_together - should be == to Totpixel', ROI_Tot_BW_together)
        #   print('GRAY_pixel - should be 0 !', GRAY_pixel)
        print('ROI_Tot_colored_pix_Perc', ROI_Tot_colored_pix_Perc)
        print('ROI_Tot_B_pix_Perc', ROI_Tot_B_pix_Perc)
        print('ROI_Tot_BW_tog_Perc - should be 100%', ROI_Tot_BW_tog_Perc)
        print('--------------------------------- ')
        print(' ')
         
        ######### print values on the screen #########
        clone_bg_removed = bg_removed.copy()
        clone_bg_removed = cv2.cvtColor(bg_removed, cv2.COLOR_BGR2RGB)  
        text1 = "ROI_Porosity = " + str(round(ROI_Tot_B_pix_Perc,1)) +"%"
        cv2.putText(clone_bg_removed, text1, (10,50),cv2.FONT_HERSHEY_COMPLEX, 1 , (0, 255,0), 2)  # put text in each frame 
        text2 = "TOT_Porosity = " + str(round(Tot_B_pix_Perc,1)) +"%" #uncomment for Tot_Porosity on screen  
        cv2.putText(clone_bg_removed, text2, (10,100),cv2.FONT_HERSHEY_COMPLEX, 1 , (0, 0, 255), 2)  ##uncomment for Tot_Porosity on screen
        #print ROI rectangle
        cv2.rectangle(clone_bg_removed, (ROI_X1,ROI_Y1) ,(ROI_X2,ROI_Y2), (0,255,0),2) 
        #convert BGR to RGB for visualization
        bg_removed  = cv2.cvtColor(bg_removed, cv2.COLOR_BGR2RGB)  

        ######### Show the things #########
        cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)  #cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Align Example', clone_bg_removed)
        key = cv2.waitKey(1)
       
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
             ####### CREATE A DATAFRAME WIHT THE RESULTS per Frame####
            # dict = {'ROI_Porosity': ROI_Porosity, 'TOT_Porosity': TOT_Porosity }    
            # df = pd.DataFrame(dict)    
            # print (df.head)
            # df.to_csv(r'C:\Users\Gianmy\Documents\Università\AGRARIA\1-PhD (2018 -2021)\TRIAL\3DResources\Frames_Porosity.csv', index = False)

            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()