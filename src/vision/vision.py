#! /usr/bin/env python
import rospy as ros

import cv2
from cv_bridge import CvBridge

import numpy as np
import matplotlib.pyplot as plt
from ctypes import * # convert float to uint32

import math

from robotics_project_vision import plot_points
from robotics_project_vision import object_detection as vision

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from robotics_project.srv import GetBlocks

from datetime import datetime
import os

# una volta effettuate delle modifiche non è necessario buildare nuovamente
# il progetto, almeno che non si debbano cambiare gli import

## from zed frame to world frame
ROTATIONAL_MATRIX = np.array([[ 0.     , -0.49948,  0.86632],
                            [-1.     ,  0.     ,  0.     ],
                            [-0.     , -0.86632, -0.49948]])
ROTATIONAL_MATRIX_INV = np.transpose(ROTATIONAL_MATRIX)

## zed position from world frame
ZED_WRT_WORLD = np.array([-0.4 ,  0.59,  1.4 ])

## constants for object detection with YoloV8
ERROR_ON_Y = 30             # error found empirically. The predictor translate of 25 pixel the y coordinates of the bbox

## constants for managing point cloud
BLOCK_LEVEL = 0.875         # z value for which we are sure to not intersect with the work-station/ground
ERROR_Z = 3                 # meaning min_value -> 0.001
PLOTH_GRAPHS = True


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

        self.predictor = vision.Object_Detection(model="dependencies/robotics_project_vision/best.pt")
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
        #print(imgName)
        cv2.imwrite(f'camera-rolls/{imgName}.png', image_cv2)

        #############################################################
        #### DA QUI NON RICHIAMO PIù FUNZIONI, E' CODICE DIRETTO ####
        #############################################################

        """
        obj = 1
        for prediction in predicted_objects:

            subdir = f"obj{obj}"
            prediction_sub_path = prediction_path + "/" + subdir + "/"
            os.mkdir(prediction_sub_path)

            x1 = math.floor(prediction[0])
            y1 = math.floor(prediction[1]) 
            x2 = math.floor(prediction[2])
            y2 = math.floor(prediction[3]) 
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
            plot_points.plot_3D_graph(np.array(obj_points_world), f"3D_representation", prediction_sub_path, open_preview=False)

            # slice of the graph with z=BLOCK_LEVEL
            block_border = np.array(dictionary[BLOCK_LEVEL])

            # filter the section block_border, getting the three vertexes
            point_min_x = block_border[:,0].argmin()
            point_max_x = block_border[:,0].argmax()
            point_min_y = block_border[:,1].argmin()

            plot_points.plot_2D_graph(np.array(block_border), "borders", prediction_sub_path)

            obj += 1
        
        ##print dictionary
        #count=0
        #for key, value in dictionary.items():
        #    print(f'{key}: {value}') 

        """

        #IDEA: 
        # - leggo tutta la point cloud relativa all'area contenente il piano di lavoroe
        # - creao un dizionario con chiave i valori di z (approssimati a 3 numeri dopo la virgola)
        # - "taglio" la point cloud a una data altezza al fine di evitare di considerare i punti del piano di lavoro ma non troppo alta per beccare anche i blocchi di altezza 0.31
        
        print("Starting process of zed points")
        import time
        start_time = time.time()

        pointCloud_path = f"predictions/{imgName}/pointCloud"
        os.makedirs(pointCloud_path)

        #the following values are taken as pixels from an image taken from zed-camera
        x1 = 663
        x2 = 1561
        y1 = 7 + 370
        y2 = 549 + 370
        working_table_zed = []
        for i in range(x1,x2):
            for j in range(y1,y2):
                working_table_zed.append((i,j))

        points_3d_world = []
        dictionary = {}
        for data in point_cloud2.read_points(pc2, field_names=['x','y','z'], skip_nans=True, uvs=working_table_zed):

            # trasforming the point in world frame
            point = ROTATIONAL_MATRIX.dot(data) + ZED_WRT_WORLD
            point = np.array(point)
            points_3d_world.append(point)

            if round(point[1],2)>0.15:  # avoid the higher part of the working_table
                # creating a dictionary to filter over z-value
                z=round(point[2],ERROR_Z)
                if z in dictionary:
                    tmp = dictionary[z]
                    tmp.append(point[:2])
                    dictionary[z] = tmp
                else:
                    dictionary[z] = [point[:2]]

           
        #print(f"Number of 3d points: {len(points_3d_world)}")  
        #print(f"Keys in dictionary: {len(dictionary)}") 

        data=np.array(dictionary[BLOCK_LEVEL])
        #0.8462959510564804
        #0.8462960622358322
        #0.8462960622358322
        # -> arrotondare a 0.00001

        #######################################################################
        from sklearn.cluster import DBSCAN
        # pip install numpy matplotlib scipy scikit-learn

        # Cluster Detection (aka divide the points for each lego bricks) using DBSCAN
        clustering = DBSCAN(eps=0.04, min_samples=1).fit(data)
        labels = clustering.labels_

        if PLOTH_GRAPHS:
            # Plot original data
            plt.scatter(data[:, 0], data[:, 1], c=labels)

        list_of_blocks = []
        for label in np.unique(labels):
            if label == -1:
                continue  # Skip noise points
            cluster_points = data[labels == label]

            vertex = cluster_points[:,0].argmin()
            vertex = np.array(cluster_points[vertex,:])
            point_max_y = vertex
            point_min_y = vertex
            for p in cluster_points:
                #points below the vertex
                if p[1]>vertex[1]:
                    if p[0]>=point_max_y[0]:
                        point_max_y=p
                #points above the vertex
                elif p[1]<vertex[1]:
                    if p[0]>=point_min_y[0]:
                        point_min_y=p
                #NOTE: potrebbe succedere che p[1]==vertex[1]. Questo è il caso in cui il blocco ha un lato parallelo all'asse y 

            v1 = point_min_y - vertex
            magnitude1 = np.linalg.norm(v1)
            v2 = point_max_y - vertex
            magnitude2 = np.linalg.norm(v2)

            # keep v1 as the biggest vector between v1 and v2
            if magnitude2 > magnitude1:
                tmp = v1
                v1 = v2
                v2 = tmp

                tmp = magnitude1
                magnitude1 = magnitude2
                magnitude2 = tmp

            yaw = np.arctan2(v1[1],v1[0])
            if PLOTH_GRAPHS:
                plt.quiver(vertex[0], vertex[1], v1[0], v1[1], angles='xy', scale_units='xy', scale=1, color='r', label='V1')
                plt.quiver(vertex[0], vertex[1], v2[0], v2[1], angles='xy', scale_units='xy', scale=1, color='b', label='V2')

            mid = (point_max_y + point_min_y) / 2
            if PLOTH_GRAPHS:
                plt.plot(mid[0], mid[1], 'gx')

            def calculate_angle(v1: np.ndarray, v2: np.ndarray) -> float:
                # Compute the dot product and magnitudes
                dot_product = np.dot(v1, v2)
                norm_v1 = np.linalg.norm(v1)
                norm_v2 = np.linalg.norm(v2)

                # Check for zero magnitude vectors
                if norm_v1 == 0 or norm_v2 == 0:
                    # Handle the zero vector case, e.g., return an angle farthest from 90 degrees
                    return float('inf')
                
                # Compute the cosine of the angle
                cos_theta = dot_product / (norm_v1 * norm_v2)
                
                # Clip the cosine value to be within the range [-1, 1] to avoid errors in arccos
                cos_theta = np.clip(cos_theta, -1.0, 1.0)
                
                # Compute the angle in radians and convert to degrees
                angle_rad = np.arccos(cos_theta)
                angle_deg = np.degrees(angle_rad)
                
                return angle_deg


            class ZedBlock:
                def __init__(self, vertex: np.ndarray, point_max_y: np.ndarray, point_min_y: np.ndarray, mid: np.ndarray, yaw: float, v1: np.ndarray, v2: np.ndarray):
                    self.vertex = vertex            # vertex point
                    self.a = point_max_y            # point with max y
                    self.b = point_min_y            # point with min y
                    self.mid = mid                  # x,y coordinates of the center of the block
                    self.yaw = yaw                  # yaw of the block, aka rotation w.r.t. z axis

                    self.v1 = v1                    # longest side 
                    self.m1 = round(np.linalg.norm(v1),2)    # length of longest side
                    self.v2 = v2                    # shortest side
                    self.m2 = round(np.linalg.norm(v2),2)    # length of shortest side

                    self.accuracy = abs(calculate_angle(self.v1, self.v2) - 90)

            block = ZedBlock(vertex=vertex, point_max_y=point_max_y, point_min_y=point_min_y, mid=mid, yaw=yaw, v1=v1, v2=v2)
            list_of_blocks.append(block)

        list_of_blocks = sorted(list_of_blocks, key=lambda block: block.accuracy)
        for b in list_of_blocks:
            print()
            print(f"Vertex: {b.vertex}")
            print(f"a: {b.a}")
            print(f"b: {b.b}")
            print(f"Mid: {b.mid}")
            print(f"Longest side: {b.m1}")
            print(f"Shortest side: {b.m2}")
            print(f"Yaw: {b.yaw}")
            print(f"Angle: {calculate_angle(b.v1, b.v2)}")

        
        if PLOTH_GRAPHS:
            plt.xlabel('x')
            plt.ylabel('y')
            plt.axis('equal')
            plt.gca().invert_yaxis()
            plt.gca().invert_xaxis()
            plt.savefig(f'{pointCloud_path}/2D_blocks.png')
        

        end_time = time.time()
        print(f"End process of zed points after {end_time-start_time}")
        


        ###################### OBJECT DETECTION #######################
        
        print("Starting process of object detection")
        start_time = time.time()

        # predict with the object-detection model
        prediction_path = f"predictions/{imgName}/yoloV8"
        predicted_objects = self.predictor.predict(image=image_cv2, path_to_save_prediction=prediction_path, top_crop=370, bottom_crop=130, print_to_console=True)

        end_time = time.time()
        print(f"End process of object detection after {end_time-start_time}")


        ###################### BBOX IN GRAFICO 2D #######################

        # per ciascun oggetto riconosciuto dalla rete neurale, riportiamo la bbox nel mondo 2d (visto da sopra)
        # valutiamo quindi se il centro dell'oggetto riconosciuto tramite point cloud sta in una sola bbox o più
        # se sta una sola bbox, allora siamo sicuri di quale oggetto è stato riconosciuto da Yolo
        # se il centro sta in più di una bbox, allora salto il blocco nella speranza che, rimuovendo blocchi nel mentre, la situazione si sistema

        from scipy.spatial import ConvexHull, Delaunay

        class YoloDetectionObj:
            def __init__ (self,obj_class: int, confidence: float, delaunay: Delaunay, number):
                self.obj_class = obj_class
                self.confidence = confidence
                self.delaunay = delaunay
                self.number = number

        yolo_blocks = []
        for prediction in predicted_objects:
            x1 = math.floor(prediction[0])
            y1 = math.floor(prediction[1]) 
            x2 = math.floor(prediction[2])
            y2 = math.floor(prediction[3]) 
            confidence = prediction[4]
            obj_class = int(prediction[5])
            obj_name = prediction[6]

            borders = []
            borders.append((x1,y1))
            borders.append((x1,y2))
            borders.append((x2,y1))
            borders.append((x2,y2))

            borders_3d = self.manage_cloud.read_cloud(pc2, borders)
            borders_world_2d = []
            for point in borders_3d:
                point = ROTATIONAL_MATRIX.dot(point) + ZED_WRT_WORLD
                point = np.array(point[:2])
                borders_world_2d.append(point)

            borders_world_2d = np.array(borders_world_2d)

            delaunay = Delaunay(borders_world_2d)
            block_detected = YoloDetectionObj(obj_class, confidence, delaunay, predicted_objects.index(prediction))
            yolo_blocks.append(block_detected)
            
            if PLOTH_GRAPHS:
                hull = ConvexHull(borders_world_2d)
                # Plot the points
                plt.plot(borders_world_2d[:, 0], borders_world_2d[:, 1], 'o')

                # Plot the convex hull
                for simplex in hull.simplices:
                    plt.plot(borders_world_2d[simplex, 0], borders_world_2d[simplex, 1], 'k-')

                # Calculate the centroid of the Convex Hull
                hull_points = borders_world_2d[hull.vertices]
                centroid = np.mean(hull_points, axis=0)
                
                # Plot the centroid with the object number
                plt.text(centroid[0], centroid[1], predicted_objects.index(prediction), fontsize=12, ha='center', va='center')

        if PLOTH_GRAPHS:
            plt.savefig(f'{pointCloud_path}/2D_bboxes.png')

        ###################### MERGE DEI RISULTATI #######################

        #creo associazione tra gli oggetti riconosciuti da yolov8 e quelli identificati dalla point_cloud

        object_class_sizes = {
            0 : {'x' : 1, 'y' : 1},
            1 : {'x' : 1, 'y' : 2}, 
            2 : {'x' : 1, 'y' : 2}, 
            3 : {'x' : 1, 'y' : 2}, 
            4 : {'x' : 1, 'y' : 2}, 
            5 : {'x' : 1, 'y' : 3}, 
            6 : {'x' : 1, 'y' : 3}, #da controllare il 6
            7 : {'x' : 1, 'y' : 4}, 
            8 : {'x' : 1, 'y' : 4}, 
            9 : {'x' : 2, 'y' : 2}, 
            10 : {'x' : 2, 'y' : 2} 
        }

        blocks_to_take = []
        for block in list_of_blocks:
            discard = False
            bbox_id = -1
            
            for bbox in yolo_blocks:
                if bbox.delaunay.find_simplex(block.mid) >= 0:
                    if discard:  # The block is inside multiple bounding boxes
                        bbox_id = -1
                        break
                    else:
                        discard = True
                        bbox_id = yolo_blocks.index(bbox)
            
            if bbox_id != -1 and yolo_blocks[bbox_id].confidence > 0.7:
                
                block_v1_size = round(block.m1 / 0.03,0)
                block_v2_size = round(block.m2 / 0.03,0)

                expected_sizes = object_class_sizes[yolo_blocks[bbox_id].obj_class]
                expected_max_size = max(expected_sizes['x'], expected_sizes['y'])
                expected_min_size = min(expected_sizes['x'], expected_sizes['y'])

                #print(f"------------------")
                #print(f"Oggetto: {list_of_blocks.index(block)}")
                #print(f"BBox: {bbox_id}")
                #print(f"m1, m2: {block.m1}  {block.m2}")
                #print(f"norm - m1, m2: {round(block.m1 / 0.03,0)}  {round(block.m2 / 0.03,0)}")
                #print(f"yolo - m1, m2: {expected_max_size}  {expected_min_size}")
                
                if block_v1_size == expected_max_size and block_v2_size == expected_min_size:
                    print(f"-> oggetto da prendere: {list_of_blocks.index(block)}\n-> bbox: {yolo_blocks[bbox_id].number}\n")
                
        

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