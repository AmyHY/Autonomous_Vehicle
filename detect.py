#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo, NavSatFix, PointCloud
from novatel_gps_msgs.msg import Inspva 
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import os
import math
import csv



class ImageProcessor:
    def __init__(self):
        rospy.init_node('image_processor', anonymous=True)

        # Set up subscribers and publishers
        self.image_sub = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/zed2/zed_node/rgb/image_processed', Image, queue_size=10)
        self.depth_sub = rospy.Subscriber('/zed2/zed_node/depth/depth_registered', Image, self.depth_callback)
        self.camera_info_sub = rospy.Subscriber('/zed2/zed_node/rgb/camera_info', CameraInfo, self.camera_info_callback)
        self.gps_sub = rospy.Subscriber('/novatel/inspva', Inspva, self.gps_callback)

        self.path_pub = rospy.Publisher('/zed2/zed_node/path', PointCloud, queue_size=20)

        self.depth = None
        self.middle_curve = None # grayscale
        self.middle_curve_eq = None # points
        self.camera_info = None # = (camera_matrix, distortion_coeffs)
        self.gps = None # = (latitude, longitude, heading)
        self.lane_state = None
        self.waypoints = None
        self.olat       = 40.0928563
        self.olon       = -88.2359994
        self.bridge = CvBridge()

    import math

    def dist(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        distance = np.round(math.sqrt((x2 - x1)**2 + (y2 - y1)**2), 3)
        return distance

    def process_image(self, image): 
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)

        #darkened = cv2.addWeighted(gray, 1.5, np.zeros_like(gray), 0, 0)

        # Isolate white from HLS to get white mask
        # lower_white = np.array([0, 80, 0], dtype=np.uint8)
        # upper_white = np.array([255, 255, 255], dtype=np.uint8)

        lower_white = np.array([0, 160, 0], dtype=np.uint8)
        upper_white = np.array([255, 255, 255], dtype=np.uint8)


        white_mask = cv2.inRange(hls, lower_white, upper_white)

        masked_image = cv2.bitwise_and(gray, gray, mask=white_mask)

        _, binary_mask = cv2.threshold(masked_image, 120, 255, cv2.THRESH_BINARY)


        sigma = 0.33
        med = np.median(gray)
        lower_thresh = int(max(0, (1.0-sigma) * med))
        upper_tresh = int(min(255, (1.0+sigma) * med))

        # Apply slight Gaussian Blur
        blurred = cv2.GaussianBlur(binary_mask, (15,15), 0)

        # Apply Canny Edge Detector
        edges = cv2.Canny(blurred, lower_thresh, upper_tresh)

        # Create a mask for the top half of the image
        height, width = edges.shape[:2]
        mask = np.zeros_like(edges, dtype=np.uint8)
        mask[(height // 2):, :] = 255  # Set the top half to 255 (white)

        # Apply the mask to erase the top half of the edges image
        # erased_edges_image = cv2.bitwise_and(edges, mask)


        height, width = edges.shape


        middle_len = 0.4
        roi_vertices = np.array([[(width * 0.1, height),      # Bottom-left vertex
                          (width * (0.5 - (middle_len/2)), height * 0.6),       # Top-left vertex
                          (width * (0.5 + (middle_len/2)), height * 0.6),       # Top-right vertex
                          (width * 0.9, height)]],            # Bottom-right vertex
                        dtype=np.int32)

        roi_edges = cv2.fillPoly(np.zeros_like(edges), roi_vertices, 255)
        roi_image = cv2.bitwise_and(edges, roi_edges)
        d = 35
        kernel = np.ones((d, d), np.uint8)
        roi_image_dilated = cv2.dilate(roi_image, kernel, iterations=1)

        # lines = cv2.HoughLines(roi_image_dilated, 1, np.pi / 180, threshold=200)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(roi_image_dilated, connectivity=8)

        # Set a threshold for the minimum size of connected components (adjust as needed)
        min_component_size = 10000

        # Filter connected components based on size
        filtered_indices = [index for index, size in enumerate(stats[1:, -1], start=1) if size > min_component_size]

        # Take the top two components based on size
        top_two_indices = sorted(filtered_indices, key=lambda index: stats[index, -1], reverse=True)[:2]

        # Create a binary mask for the top two components
        top_two_mask = np.zeros_like(labels, dtype=np.uint8)
        for i, index in enumerate(top_two_indices, start=2):  # Start enumeration from 2
            top_two_mask[labels == index] = i

        roi_image_colored = cv2.cvtColor(roi_image_dilated, cv2.COLOR_GRAY2BGR)

        # Draw bounding boxes around the top two components
        for index in top_two_indices:
            x, y, w, h = stats[index, :4]
            cv2.rectangle(roi_image_colored, (x, y), (x + w, y + h), (0, 255, 0), 2)  # BGR color: (0, 255, 0) for green, 2 is the thickness

    
        # Create a colored output image
        colored_image = cv2.applyColorMap(top_two_mask * 30, cv2.COLORMAP_JET)

          # Display the image with bounding boxes
        # cv2.imshow('Top Two Components', roi_image_colored)
        # cv2.imshow('Top Two Components', colored_image)
        # cv2.waitKey(1)
        

        # Create separate masks for the top two components
        mask1 = np.zeros_like(labels, dtype=np.uint8)
        mask2 = np.zeros_like(labels, dtype=np.uint8)

        for i, index in enumerate(top_two_indices, start=2):
            if i == 2:
                mask1[labels == index] = 255
            elif i == 3:
                mask2[labels == index] = 255

        _, filterd_roi_dilated = cv2.threshold(cv2.cvtColor(colored_image, cv2.COLOR_BGR2GRAY), 20, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(filterd_roi_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Create an empty image to draw contours
        middle_contour = np.zeros_like(image) # contains middle-curve only for future use
        lanes_array = [] 
        slopes_array = []
        lanes_pos = []
        # top_slopes_array = []
        curve_array = []

        # Draw representative points along contours and fit lines
        for idx, contour in enumerate(contours):
            # Simplify the contour
            epsilon = 0.01 * cv2.arcLength(contour, True)
            simplified_contour = cv2.approxPolyDP(contour, epsilon, True)

            if len(simplified_contour) > 1:
               
                x = simplified_contour[:, 0, 0]
                y = simplified_contour[:, 0, 1]
                
                degree_of_polynomial = 2  # Adjust the degree as needed
                coefficients = np.polyfit(y, x, degree_of_polynomial)
                slope = coefficients[1]
                curve = 2 * coefficients[0]
                y_range = np.linspace(min(y), max(y), 100)
                x_values = np.polyval(coefficients, y_range)

                lanes_array.append(np.column_stack((x_values, y_range)))
                slopes_array.append(slope)
                curve_array.append(curve)

                cv2.circle(filterd_roi_dilated, (int(x_values[0]), int(y_range[0])), radius=10, color=(255, 255, 255), thickness=-1)
                lanes_pos.append([int(x_values[0]), int(y_range[0])])

                # ~~~~~~ Display of the left and right curves ~~~~~~~~~
                # Draw the polynomial line on the empty image
                # for i in range(0, (len(x_values) - 1)//3):
                #     x1, y1 = int(x_values[i]), int(y_range[i])
                #     x2, y2 = int(x_values[i + 1]), int(y_range[i + 1])
                #     cv2.line(filterd_roi_dilated, (x1, y1), (x2, y2), (150, 150, 150), 2)


                total_delta_x = 0
                total_delta_y = 0

                for i in range(0, (len(x_values) - 1)//4):
                    x1, y1 = int(x_values[i]), int(y_range[i])
                    x2, y2 = int(x_values[i + 1]), int(y_range[i + 1])

                    # Draw the line
                    cv2.line(filterd_roi_dilated, (x1, y1), (x2, y2), (150, 150, 150), 2)
            
                    # Calculate changes in x and y
                    delta_x = x2 - x1
                    delta_y = y2 - y1

                    # Accumulate total changes
                    total_delta_x += delta_x
                    total_delta_y += delta_y

                # Calculate the overall slope
                if total_delta_x != 0:
                    overall_slope = total_delta_y / total_delta_x
                else:
                    # Handle the case where total_delta_x is 0 to avoid division by zero
                    overall_slope = float('inf')

                # print("Overall Slope:", overall_slope)
                # top_slopes_array.append(overall_slope)

        
        # print(np.shape(slopes_array))

        # for idx, slope in enumerate(slopes_array) : 
        #     print(idx, ": ", slope)

        thresh_dist = 275

        if len(lanes_array) == 1 : 
            if slopes_array[0] < 0 and self.lane_state != "right": 
                # missing right lane
                x_values_middle, y_range_middle = self.generate_middle_curve(lanes_array[0], None, width, height, False)
            elif self.lane_state != "left": 
                # missing left lane
                x_values_middle, y_range_middle = self.generate_middle_curve(None, lanes_array[0], width, height, False)
        else : 

            lane_dist = self.dist(lanes_pos[0], lanes_pos[1])
            if (lane_dist > thresh_dist) : 
                print("Turn detected!")

                slopes_abs = np.abs(slopes_array)
                min_slope_idx = np.argmin(np.abs(slopes_abs))
                cv2.circle(filterd_roi_dilated, (int(lanes_pos[min_slope_idx][0]), int(lanes_pos[min_slope_idx][1])), radius=40, color=(255, 255, 255), thickness=-1)
                
                # if slopes_array[min_slope_idx] < 0 and self.lane_state != "right": 
                # # missing right lane
                #     print("using left lane for anchor")
                #     x_values_middle, y_range_middle = self.generate_middle_curve(None, lanes_array[min_slope_idx], width, height, True)
                # else : 
                #     # missing left lane
                #     print("using right lane for anchor")
                #     x_values_middle, y_range_middle = self.generate_middle_curve(lanes_array[min_slope_idx], None, width, height, True)

            else : 
                print("No turn...")
            x_values_middle, y_range_middle = self.generate_middle_curve(lanes_array[0], lanes_array[1], width, height, False) # no issues here with middle curve 

        start_pt = len(x_values_middle) - 10
        x_left = int(width * 0.2)
        x_right = int(width * 0.8)

        if x_values_middle[start_pt] > x_left and x_values_middle[start_pt] < x_right : 
            
            # draw the new prediction, update self.middle curve
            for i in range(len(x_values_middle) - 1):
                x1_middle, y1_middle = int(x_values_middle[i]), int(y_range_middle[i])
                x2_middle, y2_middle = int(x_values_middle[i + 1]), int(y_range_middle[i + 1])
                # if i > start_pt :
                #     cv2.circle(filterd_roi_dilated, (x1_middle, y1_middle), radius=10, color=(255, 255, 255), thickness=-1)
                cv2.line(filterd_roi_dilated, (x1_middle, y1_middle), (x2_middle, y2_middle), (255, 255, 255), 3)
                cv2.line(middle_contour, (x1_middle, y1_middle), (x2_middle, y2_middle), (255, 255, 255), 3)
            self.middle_curve_eq = [x_values_middle, y_range_middle]
            self.middle_curve = cv2.cvtColor(middle_contour, cv2.COLOR_BGR2GRAY)

            # cv2.imshow("middle", self.middle_curve)
            # cv2.waitKey(1)

        else : 
            
            # skip the current line, take the previous prediction instead
            for i in range(len(x_values_middle) - 1):
                x1_middle, y1_middle = int(self.middle_curve_eq[0][i]), int(self.middle_curve_eq[1][i])
                x2_middle, y2_middle = int(self.middle_curve_eq[0][i+1]), int(self.middle_curve_eq[1][i+1])
                # if i > start_pt :
                #     cv2.circle(filterd_roi_dilated, (x1_middle, y1_middle), radius=10, color=(255, 255, 255), thickness=-1)
                cv2.line(filterd_roi_dilated, (x1_middle, y1_middle), (x2_middle, y2_middle), (255, 255, 255), 3)
                cv2.line(middle_contour, (x1_middle, y1_middle), (x2_middle, y2_middle), (255, 255, 255), 3)

        return filterd_roi_dilated # cv2.cvtColor(roi_image_dilated, cv2.COLOR_BGR2GRAY)
        
  
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Perform image manipulations (e.g., resizing, color conversion, etc.)
            # Modify the image processing steps according to your requirements
        
            processed_image = self.process_image(cv_image); 

            # Convert the processed image back to ROS Image message
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, 'mono8')

            # Publish the processed image to a new topic
            self.image_pub.publish(processed_msg)

            # print(self.waypoints)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))


    def depth_callback(self, msg):
        try:
            depth_array = np.frombuffer(msg.data, dtype=np.float32)
            depth_image = depth_array.reshape((msg.height, msg.width))
            if (self.middle_curve is not None):
                self.depth = self.filter_depth_matrix(depth_image, self.middle_curve)
                if (self.camera_info is not None):
                    self.waypoints = self.generate_waypoints()            
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))

    def filter_depth_matrix(self, depth_image, mask):
        roi_depth = np.zeros_like(depth_image)
        roi_depth = np.where((mask > 0), depth_image, 0)
        return roi_depth

    def generate_middle_curve(self, contour_points_left, contour_points_right, screen_width, screen_height, anchor):
        
        # test divergence

        # divergence = []

        # if (contour_points_left is not None and contour_points_right is not None) : 
        #     contour_points_middle = np.zeros_like(contour_points_left)
        #     # get 100 points from the middle curve (between the left and right curve)
        #     for i in range(0, len(contour_points_left), 25):
        #         x_left, y_left = contour_points_left[i]
        #         x_right, y_right = contour_points_right[i]
        #         # cv2.circle(self.middle_curve, (int(x_left), int(y_left)), radius=20, color=(255, 255, 255), thickness=-1)
        #         # cv2.circle(self.middle_curve, (int(x_right), int(y_right)), radius=20, color=(255, 255, 255), thickness=-1)
        #         divergence.append(np.abs(x_right - x_left))

        #     # image_size = (800, 600, 3)  # width, height, and channels
        #     # image = np.zeros(image_size, dtype=np.uint8)

        #     # # Normalize array values to fit within the image height
        #     # normalized_values = (divergence - np.min(divergence)) / (np.max(divergence) - np.min(divergence))
        #     # normalized_values = (image.shape[0] - 1) * normalized_values

        #     # # Draw lines connecting the points
        #     # for i in range(len(normalized_values) - 1):
        #     #     pt1 = (i * 100, int(normalized_values[i]))
        #     #     pt2 = ((i + 1) * 100, int(normalized_values[i + 1]))
        #     #     cv2.line(image, pt1, pt2, (255, 255, 255), 2)

        #     # Display the image
        #     cv2.imshow("Plot", self.middle_curve)
        #     cv2.waitKey(1)

        if anchor : 
            shift_amount = 200
        else : 
            shift_amount = 250
        
        if (contour_points_left is None and contour_points_right is None):
            # print("missing both lines")
            self.lane_state = "none"
            return [], []
        elif (contour_points_left is None):
            # print("missing left lane")
            self.lane_state = "right"
            # shift_amount = 300  # Adjust as needed
            contour_points_middle = np.zeros_like(contour_points_right)
            for i in range(len(contour_points_right)):
                x_right, y_right = contour_points_right[i]
                x_middle = x_right - shift_amount
                contour_points_middle[i] = (x_middle, y_right)
        elif (contour_points_right is None):
            # print("missing right lane")
            self.lane_state = "left"
            # shift_amount = 300  # Adjust as needed
            contour_points_middle = np.zeros_like(contour_points_left)
            for i in range(len(contour_points_left)):
                x_right, y_right = contour_points_left[i]
                x_middle = x_right + shift_amount
                contour_points_middle[i] = (x_middle, y_right)
        else:
            # print("Both lanes detected")
            self.lane_state = "both"
            contour_points_middle = np.zeros_like(contour_points_left)
            # get 100 points from the middle curve (between the left and right curve)
            for i in range(len(contour_points_left)):
                x_left, y_left = contour_points_left[i]
                x_right, y_right = contour_points_right[i]
                x_middle = (x_left + x_right) / 2
                y_middle = (y_left + y_right) / 2
                contour_points_middle[i] = (x_middle, y_middle)

        # polyfit the middle curve
        middle_point_count = len(contour_points_middle) // 2
        x_middle = np.array([point[0] for point in contour_points_middle[:middle_point_count]])
        y_middle = np.array([point[1] for point in contour_points_middle[:middle_point_count]])
        bottom_center_x = screen_width // 2
        bottom_center_y = screen_height
        x_middle = np.append(x_middle, bottom_center_x)
        y_middle = np.append(y_middle, bottom_center_y)

        degree_of_polynomial = 2
        coefficients_middle = np.polyfit(y_middle, x_middle, degree_of_polynomial)

        # Connect the middle lane to the bottom center of the screen
        y_range = np.linspace(min(y_middle), max(y_middle), 100)
        x_values = np.polyval(coefficients_middle, y_range)

        return x_values, y_range



    def camera_info_callback(self, msg):
        # Access camera matrix and distortion coefficients
        camera_matrix = np.array(msg.K).reshape((3, 3))
        distortion_coeffs = msg.D
        self.camera_info = (camera_matrix, distortion_coeffs)


    def generate_waypoints(self):
        
        waypoints = []

        # print("middle curve eq: " + str(self.middle_curve_eq))
        # print("gps: " + str(self.gps))
        # print("cam info: " + str(self.camera_info))

        if ((self.depth is None) \
            or (self.middle_curve_eq is None) \
            or (self.camera_info is None) \
            or (self.gps is None)):
            print("Missing Sensor")
            return

        
            
        # Obtain an array of all the (x, y) where depth_matrix[y][x] is larger than 0
        depth_matrix = self.depth
        step = 25

        XYD_matrix = []
        for i in range(0, len(self.middle_curve_eq[0])-1, step):
            x = self.middle_curve_eq[0][i]
            y = self.middle_curve_eq[1][i]
            if (int(y) >= depth_matrix.shape[0]) or (int(x) >= depth_matrix.shape[1]):
                return []
            depth = depth_matrix[int(y)][int(x)]
            XYD_matrix.append([x, y, depth])
            # if (self.middle_curve is not None) : 
            #     cv2.circle(self.middle_curve, (int(x), int(y)), radius=20, color=(255, 255, 255), thickness=-1)
            # Print the values for each iteration
            # print(f"Point {i + 1}: X = {x}, Y = {y}, Depth = {depth}")

        # cv2.imshow("middle", self.middle_curve)
        # cv2.waitKey(1)

        XYD_matrix = np.array(XYD_matrix)
        XYD_matrix = XYD_matrix[::-1]

        camcal_matrix = self.camera_info[0]
        fx, cx, fy, cy = camcal_matrix[0][0], camcal_matrix[0][2], camcal_matrix[1][1], camcal_matrix[1][2]

        # print(camcal_matrix)
        # print(str(fx) + " " + str(cx))
        # print(str(fy) + " " + str(cy))
        # print("\n")

        # https://stackoverflow.com/questions/51346929/how-to-convert-2dx-y-coordinates-to-3dx-y-z-coordinates-using-matlab

        # print(np.shape(XYD_matrix))
        # print("\n")

        lat = self.gps[0]
        lon = self.gps[1]
        azimuth = self.gps[2]

        # print(self.gps)

        XYZ_world_mat = []
        lat_lon_mat = []

        # append the base path point

        # print("curr loc = " + str(self.wps_to_local_xy(lon, lat)))

        cur_loc = self.wps_to_local_xy(lon, lat)
        XYZ_world_mat.append(np.array([cur_loc[0], cur_loc[1]]))

        
        for i in range(np.shape(XYD_matrix)[0]) : 
            
            x_pix = XYD_matrix[i][0]
            y_pix = XYD_matrix[i][1]
            depth = XYD_matrix[i][2]
            x_rel_world = depth * (x_pix - cx) / fx
            y_rel_world = 0 # depth * (y_pix - cy) / fy looks like Y is the "up" direction, we can null this out
            z_rel_world = depth
        
            theta = -azimuth  # You can change this to your desired angle

            rotation_matrix = np.array([[np.cos(np.radians(theta)), -np.sin(np.radians(theta))],
                                        [np.sin(np.radians(theta)), np.cos(np.radians(theta))]])
            
            # rotation_matrix = np.array([[-1, 0],[0, 1]])

            # unrotated path pts
            pt_rel_car = np.array([x_rel_world, z_rel_world])
            pt_rot_car = np.dot(rotation_matrix, pt_rel_car)
            rotated_rel_coords = pt_rot_car

            # append the unrotated path points
            # XYZ_world_mat.append(np.array([cur_loc[0] + pt_rel_car[0], cur_loc[1] + pt_rel_car[1]]))

            # append the rotated path points
            XYZ_world_mat.append(np.array([cur_loc[0] + rotated_rel_coords[0], cur_loc[1] + rotated_rel_coords[1]]))
            
            
            # current location only

            # with path
            # XYZ_world_mat.append([int(cur_loc[0] + rotated_rel_coords[0]), int(cur_loc[1] + rotated_rel_coords[1])])
            # XYZ_world_mat.append([int(pt_rel_car[0] + rotated_rel_coords[0]), int(pt_rel_car[1] + rotated_rel_coords[1])])

        # print(f"Point {i + 1}: X = {cur_loc[0] + rotated_rel_coords[0]}, Y = {y_rel_world}, Z = {cur_loc[0] + rotated_rel_coords[0]}")

            # R=6378137                                # Earth’s radius, sphere
            # dn = x_rel_world                         # offsets in meters
            # de = z_rel_world
            # dLat = dn/R                              # Coordinate offsets in radians
            # dLon = de/(R*np.cos(np.pi*lat/180))
            # latO = lat + dLat * 180/np.pi            # OffsetPosition, decimal degrees
            # lonO = lon + dLon * 180/np.pi 
            # lat_lon_mat.append([latO, lonO])
            # #print(f"Point {i + 1}: Lat = {latO}, Lon = {lonO}")
            # lon_wp_x, lat_wp_y = self.wps_to_local_xy(lonO, latO)
            # # print(f"Point {i + 1}: Lon = {lon_wp_x}, Lat = {lat_wp_y}")
            # waypoints.append([lon_wp_x, lat_wp_y, 0])

        

        # waypoints = np.array(waypoints)
        # waypoints = waypoints[::-1]

        # for i, point in enumerate(XYZ_world_mat) :
        #     lo = point[0]
        #     la = point[1]
            # Find relative heading between points, taking into account 
            # if (i==0) :
            #     prev_head = self.gps[2]
            # else: 
            #     prev_head = waypoints[i-1]
            

            # print(f"Point {i + 1}: Lon = {lo}, Lat = {la}, Head = {self.gps[2]}")



        # diffs = waypoints[:, :2] - waypoints[0:1, :2]  # Calculate differences between each waypoint and the first waypoint
        # relative_heading = np.arctan2(diffs[:, 1], diffs[:, 0])  # Calculate the angle using arctan2
        # relative_heading = np.degrees(relative_heading)  # Convert angles to degrees if needed


        # gps_heading = relative_heading + theta
        # waypoints[:, 2] = gps_heading
        # np.savetxt("sampled_waypoints_new.csv", waypoints, delimiter=",",  comments="")    
        np.savetxt("XYZ_points.csv", XYZ_world_mat, delimiter=",",  comments="")    




        # Construct waypoints
        XYZ_world_mat_array = np.array(XYZ_world_mat)
        waypoints = np.zeros((XYZ_world_mat_array.shape[0], 3))
        waypoints[:, :2] = XYZ_world_mat_array[:, :2]

        # Method 1 : get heading relative to current position
        # for waypoint in waypoints:
        #     deltaX = waypoint[0] - cur_loc[0]
        #     deltaY = waypoint[1] - cur_loc[1]
        #     wpt_heading =  (np.degrees(- np.arctan2(deltaY, deltaX)) + 90) % 360 # flip and rotate the radian plane
        #     waypoint[2] =  wpt_heading

        # Method 2 : get heading relative to previous waypoint
        for i in range(len(waypoints) - 1):
            waypoint1 = waypoints[i]
            waypoint2 = waypoints[i + 1]
            deltaX = waypoint2[0] - waypoint1[0]
            deltaY = waypoint2[1] - waypoint1[1]
            wpt_heading =  (np.degrees(- np.arctan2(deltaY, deltaX)) + 90) % 360 # flip and rotate the radian plane
            waypoints[i + 1][2] =  wpt_heading
            if i == 0:
                waypoints[0][2] = waypoints[1][2]
        
        # Filter out rows containing 'inf' or 'nan'
        valid_rows = ~np.any(np.isinf(waypoints) | np.isnan(waypoints), axis=1)
        filtered_waypoints = waypoints[valid_rows]

        # Save the filtered waypoints to CSV
        np.savetxt("waypoints.csv", waypoints, delimiter=",", fmt='%f', comments="")
        




        # print("\n")
        # print("\n")
        # print("waypoints = " + str(waypoints[10]))
        # print("num of waypoints = " + str(waypoints.shape[0]))
        # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

        # Output to file
        # print("here")
        # with open("base.csv", mode='a', newline='') as file: 
        #     csv_writer = csv.writer(file)
        #     csv_writer.writerow(self.wps_to_local_xy(lon, lat))

            # csv_writer.writerow(self.wps_to_local_xy(lon, lat))

        # np.savetxt("wpt0.csv", str(waypoints[0]), delimiter=",",  comments="")    

        # print(waypoints)

        #return XYZ_world_mat
        return XYZ_world_mat

    def wps_to_local_xy(self, lon_wp, lat_wp):
        # convert GNSS waypoints into local fixed frame reprented in x and y
        lon_wp_x, lat_wp_y = self.ll2xy(lat_wp, lon_wp, self.olat, self.olon)
        return lon_wp_x, lat_wp_y

    def ll2xy(self, lat, lon, orglat, orglon):
        # Converts Lat/Long to X/Y

        # Returns:
        #     tuple: (x,y) where...
        #         x is Easting in m (Alvin local grid)
        #         y is Northing in m (Alvin local grid)

        latrad1 = orglat*2.0*(math.pi)/360.0 
        mdeglon = 111415.13 * math.cos(latrad1) \
            - 94.55 * math.cos(3.0*latrad1) \
            + 0.12 * math.cos(5.0*latrad1)
        

        latrad2 = orglat*2.0*(math.pi)/360.0
        mdeglat = 111132.09 - 566.05 * math.cos(2.0*latrad2) \
            + 1.20 * math.cos(4.0*latrad2) \
            - 0.002 * math.cos(6.0*latrad2)

        x = (lon - orglon) * mdeglon
        y = (lat - orglat) * mdeglat
        return (x,y)

    def gps_callback(self, msg):
        self.gps = (msg.latitude, msg.longitude, msg.azimuth)
        # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        # print("latitude = " + str(self.gps[0]))
        # print("longitude = " + str(self.gps[1]))
        # print("heading = " + str(self.gps[2]))
        


if __name__ == '__main__':
    try:
        image_processor = ImageProcessor()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass