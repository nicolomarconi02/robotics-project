#! /usr/bin/env python
import rospy as ros

import cv2
from cv_bridge import CvBridge

from ultralytics import YOLO

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from ctypes import * # convert float to uint32

import math

## da rimuovere
import plot_points as plot_points

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from robotics_project.srv import GetBlocks

from datetime import datetime

# una volta effettuate delle modifiche non è necessario buildare nuovamente
# il progetto, almeno che non si debbano cambiare gli import

PATH_TO_IMAGES = "camera-rolls/"
PATH_TO_PREDICTIONS = "predictions/"
MODEL = "src/vision/best.pt"

CONFIDENCE_THRESHOLD = 0.5
INTERSECTION_OVER_UNION_THRESHOLD = 0.5
BLOCK_LEVEL = 0.887         # z value for which we are sure to not intersect with the work-station/ground
ERROR_Z = 3                 # meaning min_value -> 0.001
ERROR_ON_Y = 30             # error found empirically. The predictor translate of 25 pixel the y coordinates of the bbox

# from zed frame to world frame
ROTATIONAL_MATRIX = np.array([[ 0.     , -0.49948,  0.86632],
                            [-1.     ,  0.     ,  0.     ],
                            [-0.     , -0.86632, -0.49948]])
        
# zed position from world frame
ZED_WRT_WORLD = np.array([-0.4 ,  0.59,  1.4 ])

class Object_Detection():
    def __init__(self):

        self.model = YOLO(MODEL)

    def print(self, list):
        print("\n********************\nObjects found:")
        for block in list:
            print(f"- {block[6]} ({(int(block[5]))}): box->{block[:4]}, confidence->{block[4]}")
        print("********************\n")

    def predict(self, image : Image, name_file, print=False, height=1080, width=1920, bottom_crop=0, top_crop=0, right_crop=0, left_crop=0):
        
        #pre-processing of the image (cropped eliminating 370 pixels from the top in order to remote the UR5-arm)
        cropped_image = image[top_crop:height-bottom_crop, left_crop:width-right_crop]
        cv2.imwrite(f"{PATH_TO_PREDICTIONS}{name_file}.png", cropped_image)

        #prediction with the model
        results = self.model(cropped_image, imgsz=640, conf=CONFIDENCE_THRESHOLD, iou=INTERSECTION_OVER_UNION_THRESHOLD)[0]
        
        boxes = results.boxes.xyxy.tolist()
        classes = results.boxes.cls.tolist()
        names = results.names
        confidences = results.boxes.conf.tolist()

        info_file = PATH_TO_PREDICTIONS + name_file + "_info.txt"
        f = open(info_file,"w")

        # Iterate through the results
        i=1
        prediction_list = []
        for box, cls, conf in zip(boxes, classes, confidences):
            x1, y1, x2, y2 = box

            y1 += 7
            y2 += 7
            
            name = names[int(cls)]

            label = f"{name} {conf:.2f}"
            cv2.rectangle(cropped_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(cropped_image, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            x1 += left_crop
            x2 += left_crop
            y1 += top_crop
            y2 += top_crop

            prediction_list.append([x1,y1,x2,y2,conf,cls,name])
            
            f.write(f"OBJ-{i}\nbox: [{x1},{y1},{x2},{y2}]\nconf: {conf}\ndetected_class: {cls}\nname: {name}\n\n*************************************************\n\n")
            i+=1

        f.close()            

        # Save the resulting image with bounding boxes
        prediction_file = f"{PATH_TO_PREDICTIONS}{name_file}_prediction.jpg"
        cv2.imwrite(prediction_file, cropped_image)

        if print:
            self.print(prediction_list)

        return prediction_list

class Manage_Point_Cloud():
    def __init__(self):
        pass
        
    def extract_data(self, point_cloud2_msg : PointCloud2):
        self.point_cloud2_msg = point_cloud2_msg
    
    def read_cloud(self, pc2, points_2d):
        points_3d = []
        for data in point_cloud2.read_points(pc2, field_names=['x','y','z'], skip_nans=False, uvs=points_2d):
            points_3d.append([data[0], data[1], data[2]])
        return points_3d

class VisionManagerClass():

    def __init__(self, robot_name="ur5"):
        ros.init_node('vision')

        self.predictor = Object_Detection()
        self.manage_cloud = Manage_Point_Cloud()
        
        # Image from ZED Node
        self.image_msg = ros.wait_for_message("/ur5/zed_node/left_raw/image_raw_color", Image)

        # Point cloud from ZED Node
        self.point_cloud2_msg = ros.wait_for_message("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2)

        # Make use of the image obtained
        self.digest_ZED_data(self.image_msg, self.point_cloud2_msg)

        return
        # Service's definition and its handler setting
        s = ros.Service('GetBlocks', GetBlocks, self.handle_get_blocks)

        # The node runs indefinitely
        ros.spin()


    def digest_ZED_data(self, image : Image, pc2 : PointCloud2):
        """
        At the moment this function is unfinished, because we still can't send the data
        to the clients.
    
        Reference to this temporary function's content:
        https://github.com/mfocchi/robot_control/blob/0efd6d8298849e2b4d8ecb88f3e256940ede2be5/lab_exercises/lab_palopoli/ur5_generic.py#L258

        This function digests the data obtained from the camera.
        Arguments:
            image: the image itself
            pc2: point cloud representation
        Returns:
            Nothing as now
        """

        # convert received image (bgr8 format) to a cv2 image
        image_cv2 = CvBridge().imgmsg_to_cv2(image, "bgr8")
        imgName = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        print(imgName)
        cv2.imwrite(f'{PATH_TO_IMAGES}/{imgName}.png', image_cv2)

        # predict with the object-detection model
        predicted_objects = self.predictor.predict(image_cv2, imgName, top_crop=370, print=True)
        

        #############################################################
        #### DA QUI NON RICHIAMO PIù FUNZIONI, E' CODICE DIRETTO ####
        #############################################################


        #for prediction in predicted_objects:
        prediction = predicted_objects[0]
        x1 = math.floor(prediction[0])
        y1 = math.floor(prediction[1]) #- ERROR_ON_Y
        x2 = math.floor(prediction[2])
        y2 = math.floor(prediction[3]) #+ ERROR_ON_Y
        confidence = prediction[4]
        obj_class = int(prediction[5])
        obj_name = prediction[6]

        # iteration over all the area of interest, getting 2D points
        obj_points_2d = []
        for i in range(x1,x2):
            for j in range(y1,y2):
                obj_points_2d.append((i,j)) 
        
        # getting 3D points from point cloud
        obj_points_3d = self.manage_cloud.read_cloud(pc2, obj_points_2d)
        print(len(obj_points_3d))

        # trasforming each point in world frame
        obj_points_world = []
        dictionary = {}
        for point in obj_points_3d:
            point = ROTATIONAL_MATRIX.dot(point) + ZED_WRT_WORLD
            point = np.array(point)
            obj_points_world.append(point)

            # creating a dictionary to filter over z-value
            z = round(point[2],ERROR_Z)
            if z in dictionary:
                tmp = dictionary[z]
                tmp.append(point[:2])
                dictionary[z] = tmp
            else:
                dictionary[z] = [point[:2]]

        ## plot 3D graph of the block
        plot_points.plot_graph(np.array(obj_points_world))

        # slice of the graph with z=BLOCK_LEVEL
        block_border = np.array(dictionary[BLOCK_LEVEL])

        # filter the section block_border, getting the three vertexes
        point_min_x = block_border[:,0].argmin()
        point_max_x = block_border[:,0].argmax()
        point_min_y = block_border[:,1].argmin()

        plot_points.plot_graph_2d(np.array(block_border))

        
        ##print dictionary
        #count=0
        #for key, value in dictionary.items():
        #    print(f'{key}: {value}')        
        
        return    


    def handle_get_blocks():
        """
        This function gets called in order to reply to the service's calls coming from the clients.
        Arguments:
            Nothing
        Returns:
            Nothing
        """

        print('VISION PROCESS: handle_obtain_blocks CALLED')


def main():
    print('VISION PROCESS: STARTED')

    # Starting the ROS Node and keep it running
    node = VisionManagerClass()


if __name__ == '__main__':
    main()