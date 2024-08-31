#! /usr/bin/env python

"""!
Starts a ROS service that takes a sensor_msgs/Image and a sensor_msgs/pc2 as input, elaborates the data in order to detect 
and locate the blocks on the working table, and returns geometry_msgs/Pose[] as output.

The blocks are detected via a YoloV8 model, from which a prediction (and its confidence) is obtained. The point 
cloud is used to check and verify the prediction of the model, as well as to locate the block on the table with 
(x, y, z) coordinates with respect to the world frame.

The blocks that are returned in geometry_msgs/Pose[] as output are not always all the detected ones, but only the 
ones with a high probability of being correctly identified. This is why the service may need to be run multiple 
times until the boolean geometry_msgs/Pose[]/finished is returned as True.
"""

import rospy as ros

import cv2
from cv_bridge import CvBridge

import numpy as np
import matplotlib.pyplot as plt
from ctypes import *    # To convert float to uint32

import math
import time
import threading

from robotics_project_vision import object_detection as vision

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from robotics_project.srv import GetBlocks, GetBlocksResponse

from datetime import datetime
import os
import re

from geometry_msgs.msg import Pose

import sys
sys.path.append("src/world")
from world import Models, TABLE_HEIGHT, UNIT_HEIGHT, UNIT_LENGTH


## Rotation matrix from zed frame to world frame
ROTATIONAL_MATRIX = np.array([[ 0.     , -0.49948,  0.86632],
                            [-1.     ,  0.     ,  0.     ],
                            [-0.     , -0.86632, -0.49948]])

## Zed position with respect to world frame
ZED_WRT_WORLD = np.array([-0.4 ,  0.59,  1.4 ])

## Path where the YoloV8 model (.pt) can be found 
MODEL = "dependencies/robotics_project_vision/best.pt"           

## z-value for which we are sure to not intersect with the work-station/ground
BLOCK_LEVEL = 0.872 
## Taking only ERROR_Z decimals in the z-values to create the key of the point-cloud-dictionary       
ERROR_Z = 3     
## Create graphs to see the blocks detected via YoloV8 and Zed-Camera          
PLOTH_GRAPHS = True

## How to round the side of the blocks so that they are normalized to the prediction values
NORM_TRESHOLD = 0.12

## Debug flag
DEBUG = False

# move to repo
from sklearn.cluster import DBSCAN
class Manage_Point_Cloud():
    """!
    Class to manage the Point Cloud
    """
    
    def __init__(self, rotational_matrix, zed_wrt_world):

        ## 3D Point Cloud
        self.point_cloud2_msg = None

        ## Rotation matrix from ZED Camera to World
        self.rotational_matrix = rotational_matrix
        ## Transformation from ZED Camera to the World
        self.zed_wrt_world = zed_wrt_world
    
    def extract_point_world(self, points_2d, dim):
        """!
        Extracts the 3D points in world frame given a list of 2D points (pixel in the image).

        @param points_2d: list of points (pixel) in 2D
        @param dim: numer of dimensions we want to extract from the point cloud (dim=2 means (x,y), dim=3 means (x,y,z))

        @return Returns a list of 3d points in world frame
        """

        points_3d_world = []
        for data in point_cloud2.read_points(self.point_cloud2_msg, field_names=['x','y','z'], skip_nans=True, uvs=points_2d):

            # Trasforming the point in world frame
            point = self.rotational_matrix.dot(data) + self.zed_wrt_world

            if dim == 2 :
                point = np.array(point[:2])
            elif dim == 3:
                point = np.array(point)
            points_3d_world.append(point)

        return points_3d_world

def exchange (a,b):
    """!
    Exchange the a value with the b value.

    @param a: first value
    @param b: second value

    @return Returns the tuple (b,a)
    """
    return b,a

class ZedBlock:
    """!
    Class to represent the blocks detected from the analysis of the point cloud given by the Zed Camera.
    """

    def __init__(self, cluster_points=None):
        ## Cluster points
        self.cluster_points = cluster_points
        
        ## (x,y) coordinates of the vertex point
        self.vertex = None  
        self.vertex : np.ndarray      

        ## (x,y) coordinates of the point that, together with the vertex, generates the longest side 
        self.p1 = None  
        self.p1 : np.ndarray

        ## Vector that represents the longest side         
        self.v1 = None
        self.v1 : np.ndarray

        ## Length (norm) of the longest side            
        self.n1 = 0  
        self.n1 : float  

        ## (x,y) coordinates of the point that, together with the vertex, generates the shortest side 
        self.p2 = None 
        self.p2 : np.ndarray  

        ## Vector that represents the shortest side
        self.v2 = None  
        self.v2 : np.ndarray 

        ## Length (norm) of the shortest side         
        self.n2 = 0  
        self.n2 : float                  
        
        ## Angle [degrees] between the two sides (should be around 90°)
        self.angle = None 
        self.angle : float   

        ## Accuracy defined as "how far away from 90° the angle between v1 and v2 is"        
        self.accuracy = None
        self.accuracy : float           

        ## (x,y) coordinates of the center of the block
        self.mid = None  
        self.mid : np.ndarray         

        ## Yaw [radians] of the block, aka rotation w.r.t. z axis 
        self.yaw = None   
        self.yaw : float

        ## Yolo bbox id
        self.yolo_bbox_id = None
        ## Yolo confidence
        self.yolo_confidence = None
        ## Yolo prediction
        self.yolo_prediction = None

        if self.cluster_points is not None:
            self.compute_vertex()
            self.compute_sides()
            self.compute_angle()
            if self.v2 is not None:
                self.compute_mid()
                self.compute_yaw()
            else:
                self.mid = (self.p1 + self.vertex)/2
                self.mid[0] += UNIT_HEIGHT

    def get_quadrants(self, origin = None, exclude_origin = True):
        """!
        Determines the cluster points on the 4 quadrants centered on the origin.

        @param origin: origin point of the plane's division, its default value is the block's vertex
        @param exclude_origin: this boolean specifies if the origin may be accepted as a returned value or not 

        @return Returns the cluster points on the 4 quadrants centered on the origin.
        """
        if origin is None:
            origin = self.vertex[:]
        points = self.cluster_points[:]

        # The origin gets removed from the pool, so that it doesn't get count on the quadrants
        if exclude_origin:
            points = np.delete(points, np.where(points == origin)[0][0], axis=0)

        quadrants = [[], [], [], []]

        for p in points:
            if p[0] <= origin[0] and p[1] <= origin[1]:
                quadrants[0].append(p)
            if p[0] >= origin[0] and p[1] <= origin[1]:
                quadrants[1].append(p)
            if p[0] >= origin[0] and p[1] >= origin[1]:
                quadrants[2].append(p)
            if p[0] <= origin[0] and p[1] >= origin[1]:
                quadrants[3].append(p)
    
        return quadrants

    def reduce_point_on_direction(self, point, quadrant, axis, extreme, exclude_origin=True):
        """!
        This function finds the furthest/closest point in a quadrant, starting from a given point, relative to an axis
        
        @param point: starting point of the research
        @param quadrant: quadrant to analyze
        @param axis: axis in which we move to search, either x (=0) or y (=1)
        @param extreme: it specifies whether we're going to search for an argmin (=0) or an argmax (=1)
        @param exclude_origin: this boolean specifies if the origin may be accepted as a returned value or not 
        
        @return The found point, or None.
        """
        quadrant = np.array(self.get_quadrants(point, exclude_origin)[quadrant])

        # it may happen with exclude_origin = True
        if len(quadrant) == 0:
            return None

        if extreme == 0:
            return np.array(quadrant[quadrant[:,axis].argmin(),:])
        return np.array(quadrant[quadrant[:,axis].argmax(),:])

    def has_close_point(self, point, quadrant, axis, extreme, max_distance = 0.001):
        """!
        This function checks if there's a close point in the given direction of a point's quadrant
        
        @param point: starting point of the research
        @param quadrant: quadrant to analyze
        @param axis: axis in which we move to search, either x (=0) or y (=1)
        @param extreme: it specifies whether we're going to search for an argmin (=0) or an argmax (=1)
        @param max_distance: distance limit, on the axis, to be considered close
        
        @return The found point, or None.
        """
        closest_point = self.reduce_point_on_direction(point, quadrant, axis, extreme, False)
        return closest_point is not None and abs(closest_point[axis] - point[axis]) <= max_distance

    def compute_vertex(self):
        """!
        Computes the vertexes of the block.

        @return Nothing, it changes the object variables.
        """
        point = self.cluster_points[:,0].argmin()
        self.vertex = np.array(self.cluster_points[point,:])

        # we get how many points are positioned in each quadrant around the vertex
        q1, q2, q3, q4 = [len(q) for q in self.get_quadrants()]

        # if only a quadrant has points, the block is near the camera and the block's angle requires a different vertex computation (y min/max)
        if (q1 == 0) + (q2 == 0) + (q3 == 0) + (q4 == 0) == 3:
            if q1 > 0 or q2 > 0:
                point = self.cluster_points[:,1].argmin()
            else:
                point = self.cluster_points[:,1].argmax()
            self.vertex = np.array(self.cluster_points[point,:])


    def compute_sides(self):
        """!
        Computes the sides of the block.

        @return Nothing, it changes the object variables.
        """
        q1_points, q2_points, q3_points, q4_points = [np.array(q) for q in self.get_quadrants()]
        q1, q2, q3, q4 = [len(q) for q in [q1_points, q2_points, q3_points, q4_points]]

        # set default
        point_min = self.vertex
        point_max = self.vertex

        # the sides get set based on the vertex position
        # right vertex
        if q1 + q4 == 0 and q2 > 0 and q3 > 0:
            i = q2_points[:,0].argmax()
            j = q3_points[:,0].argmax()
            point_min = np.array(q2_points[i,:])
            point_max = np.array(q3_points[j,:])
        # bottom vertex
        if q3 + q4 == 0 and q1 > 0 and q2 > 0:
            i = q1_points[:,1].argmin()
            j = q2_points[:,1].argmin()
            point_min = np.array(q1_points[i,:])
            point_max = np.array(q2_points[j,:])
        # top vertex
        elif q1 + q2 == 0 and q3 > 0 and q4 > 0:
            i = q4_points[:,1].argmax()
            j = q3_points[:,1].argmax()
            point_min = np.array(q4_points[i,:])
            point_max = np.array(q3_points[j,:])
        # there's only a quadrant available around the vertex (single vector)
        elif (q1 == 0) + (q2 == 0) + (q3 == 0) + (q4 == 0) == 3:
            if q1 > 0:
                i = q1_points[:,0].argmin()
                point_min = np.array(q1_points[i,:])
            elif q2 > 0:
                i = q2_points[:,0].argmax()
                point_min = np.array(q2_points[i,:])
            elif q3 > 0:
                i = q3_points[:,0].argmax()
                point_min = np.array(q3_points[i,:])
            elif q4 > 0:
                i = q4_points[:,0].argmin()
                point_min = np.array(q4_points[i,:])

        # additional system: we find the potential vertexes and  check which are the three visible
        first = np.array(self.cluster_points[self.cluster_points[:,0].argmin(),:])
        second = np.array(self.cluster_points[self.cluster_points[:,0].argmax(),:])
        third = np.array(self.cluster_points[self.cluster_points[:,1].argmin(),:])
        fourth = np.array(self.cluster_points[self.cluster_points[:,1].argmax(),:])

        first_unique = not (np.array_equal(first, second) or np.array_equal(first, third) or np.array_equal(first, fourth))
        second_unique = not (np.array_equal(second, first) or np.array_equal(second, third) or np.array_equal(second, fourth))
        third_unique = not (np.array_equal(third, first) or np.array_equal(third, second) or np.array_equal(third, fourth))
        fourth_unique = not (np.array_equal(fourth, first) or np.array_equal(fourth, second) or np.array_equal(fourth, third))

        sensibility = 0.01
        if first_unique and third_unique:

            if self.has_close_point(first, 3, 1, 0):
                self.vertex = first

                third[1] += sensibility
                point_min = self.reduce_point_on_direction(third, 1, 0, 1, False)
                
                fourth[1] -= sensibility
                point_max = self.reduce_point_on_direction(fourth, 2, 0, 1, False)
            else:
                self.vertex = third

                first[0] += sensibility
                point_min = self.reduce_point_on_direction(first, 3, 1, 1, False)

                second[0] -= sensibility
                point_max = self.reduce_point_on_direction(second, 2, 1, 1, False)

        elif first_unique and fourth_unique:
            
            if self.has_close_point(first, 1, 1, 1):
                self.vertex = first

                third[1] += sensibility
                point_min = self.reduce_point_on_direction(third, 1, 0, 1, False)
                
                fourth[1] -= sensibility
                point_max = self.reduce_point_on_direction(fourth, 2, 0, 1, False)
            else:
                self.vertex = fourth

                first[0] += sensibility
                point_min = self.reduce_point_on_direction(first, 0, 1, 0, False)

                second[0] -= sensibility
                point_max = self.reduce_point_on_direction(second, 1, 1, 0, False)

        elif second_unique and fourth_unique:
            self.vertex = fourth

            first[0] += sensibility
            point_min = self.reduce_point_on_direction(first, 0, 1, 0, False)

            second[0] -= sensibility
            point_max = self.reduce_point_on_direction(second, 1, 1, 0, False)
        elif second_unique and third_unique:
            self.vertex = third

            first[0] += sensibility
            point_min = self.reduce_point_on_direction(first, 3, 1, 1, False)

            second[0] -= sensibility
            point_max = self.reduce_point_on_direction(second, 2, 1, 1, False)

        # Get the length of the sides
        p1 = point_min   
        v1 = p1 - self.vertex
        n1 = round(np.linalg.norm(v1),3)
        
        p2 = point_max
        v2 = p2 - self.vertex
        n2 = round(np.linalg.norm(v2),3)

        # Keep v1 as the longest side. If not, invert the values
        if n2 > n1:
            v1, v2 = exchange(v1,v2)
            n1, n2 = exchange(n1,n2)
            p1,p2 = exchange(p1,p2)

        # Save the values in the ZedBlock object
        self.p1 = p1
        self.v1 = v1
        self.n1 = n1

        if not np.array_equal(p2, self.vertex):
            self.p2 = p2
            self.v2 = v2
            self.n2 = n2

    def compute_angle(self):
        """!
        Computes the angle between the two sides.

        @return Nothing, it changes the object variables.
        """

        # Check for zero magnitude vectors
        if self.n1 == 0 or self.n2 == 0:
            # Handle the zero vector case, e.g., return an angle farthest from 90 degrees
            self.angle = float('inf')
            self.accuracy = float('inf')
            return
        
        # Compute the dot product and magnitudes of v1 and v2
        dot_product = np.dot(self.v1, self.v2)
        
        # Compute the cosine of the angle, and clip the cosine value to be within the range [-1, 1] to avoid errors in arccos
        cos_theta = dot_product / (self.n1 * self.n2)
        cos_theta = np.clip(cos_theta, -1.0, 1.0)
        
        # Compute the angle in radians and convert to degrees
        angle_rad = np.arccos(cos_theta)
        angle_deg = np.degrees(angle_rad)
        
        self.angle = angle_deg
        self.accuracy = abs(self.angle - 90)

    def compute_mid(self):
        """!
        Computes the (x,y) components of the middle (center) point of the block.

        @return Nothing, it changes the object variables.
        """
        self.mid = (self.p1 + self.p2) / 2

    def compute_mid_3d(self):
        """!
        Computes the z component of the middle (center) point of the block.

        @return Nothing, it changes the object variables.
        """
        self.mid = [self.mid[0], self.mid[1], (TABLE_HEIGHT + Models[self.yolo_prediction].center.height)]

    def compute_yaw(self):
        """!
        Computes the yaw of the block.

        @return Nothing, it changes the object variables.
        """
        self.yaw = np.arctan2(self.v1[1],self.v1[0])
    
    def __str__(self):
        return f"ZedBlock(vertex={self.vertex}, p1={self.p1}, v1={self.v1}, n1={self.n1}, p2={self.p2}, v2={self.v2}, n2={self.n2}, angle={self.angle}, accuracy={self.accuracy}, mid={self.mid}, yaw={self.yaw}, yolo_bbox_id={self.yolo_bbox_id}, yolo_confidence={self.yolo_confidence}, yolo_prediction={self.yolo_prediction})"

    def plot(self):
        """!
        Allows the plotting of the blocks.

        @return Nothing, it changes the object variables.
        """
        # Plot v1 (longest side) in red, v2 in blue
        plt.quiver(self.vertex[0], self.vertex[1], self.v1[0], self.v1[1], angles='xy', scale_units='xy', scale=1, color='r', label='V1')
        if self.v2 is not None:
            plt.quiver(self.vertex[0], self.vertex[1], self.v2[0], self.v2[1], angles='xy', scale_units='xy', scale=1, color='b', label='V2')
        if self.mid is not None:
            plt.plot(self.mid[0], self.mid[1], 'gx')

class VisionManagerClass():
    """!
    Class to manage the Vision Process. It creates a Object_Detection and a Manage_Point_Cloud 
    instances to use the model and manage the point cloud respectively.

    It also manages the main important variables using during the vision process, such as:
    - self.zed_blocks[], which contains the list of blocks and all the information detected by the point cloud. 
    This list is actually the most important one since the information we discover about each object are saved all here
    - self.yolo_blocks[], which contains the list of blocks and all the information predicted by the yolo model
    - self.blocks_to_take[], in which the blocks correctly classified are added in order to be returned to the manipulator
    - self.has_finished_blocks, a boolean which indicates when the vision process is finished, aka there are no more blocks on the working table
    """

    def __init__(self, robot_name="ur5"):

        ## An instance of Object_Detection Class which executes the predictions with the YoloV8 model
        self.predictor = vision.Object_Detection(model=MODEL)
        ## An instance of the Manage_Point_Cloud which manages the Point Cloud
        self.manage_cloud = Manage_Point_Cloud(rotational_matrix=ROTATIONAL_MATRIX, zed_wrt_world=ZED_WRT_WORLD)
        
        ## Image from ZED Node
        self.image_msg = ros.wait_for_message("/ur5/zed_node/left/image_rect_color", Image)
        self.image_msg = ros.wait_for_message("/ur5/zed_node/left/image_rect_color", Image)

        ## Point cloud from ZED Node
        self.point_cloud2_msg = ros.wait_for_message("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2)
        
        ## List of blocks and all the information detected by the point cloud. All information about each object are saved here
        self.zed_blocks = []
        ## List of blocks and all the information predicted by the yolo model
        self.yolo_blocks = []
        ## List of blocks correctly classified that will be returned to the caller
        self.blocks_to_take = []
        ## Boolean which indicates when the vision process is finished, aka there are no more blocks on the working table
        self.has_finished_blocks = False
        plt.clf()

        # Make use of the image obtained
        self.digest_ZED_data(self.image_msg, self.point_cloud2_msg)


    def digest_ZED_data(self, image : Image, pc2 : PointCloud2):
        """!
        It digests the data obtained from the camera.
        
        @param image: the image itself
        @param pc2: point cloud representation
        
        @return Nothing, it changes the object variables.
        """

        imgName = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        # starting object detection's thread
        threadObjDetection = threading.Thread(target=self.object_detection, args=(imgName,image))
        threadObjDetection.start()


        # ############### POINT CLOUD ################

        print("> POINT CLOUD PROCESS: STARTED")
        start_time = time.time()

        pointCloud_path = f"predictions/{imgName}/pointCloud"
        os.makedirs(pointCloud_path)

        # Save PointCloud in Manage_Point_Cloud instance
        self.manage_cloud.point_cloud2_msg = pc2

        # Get point-cloud restricteed to working table. Note that the following values
        # are empirically found; they are the pixels from an image taken from the camera 
        x1 = 663
        x2 = 1561
        y1 = 377
        y2 = 919
        working_table_zed = []
        for i in range(x1,x2):
            for j in range(y1,y2):
                working_table_zed.append((i,j))

        points_3d_world = self.manage_cloud.extract_point_world(working_table_zed, dim=3)

        # Creation of a dictionary containing all the point clouds of the working area,
        # but organized with z (rounded up to 3 decimal values) as key. This allows us 
        # to "cut" the point cloud at a given high and analyze only such points
        dic_3d_world = {}
        for point in points_3d_world:
            if round(point[1],2) > 0.25:     # avoid the higher part of the working_table
                z = round(point[2],ERROR_Z)  # rounding the z-value we get a significant one
                if z in dic_3d_world:
                    tmp = dic_3d_world[z]
                    tmp.append(point[:2])
                    dic_3d_world[z] = tmp
                else:
                    dic_3d_world[z] = [point[:2]]

        try:
            data = np.array(dic_3d_world[BLOCK_LEVEL])
        except KeyError:
            self.has_finished_blocks = True
            if DEBUG:
                print("No more blocks on the working table")
            return
        
        # Dividing the points into cluster (Cluster Detection), where each cluster represents
        # a different lego brick, using DBSCAN
        clustering = DBSCAN(eps=0.04, min_samples=1).fit(data)
        labels = clustering.labels_

        if PLOTH_GRAPHS:
            # Plot original data
            plt.scatter(data[:, 0], data[:, 1], c=labels)

        # For each cluster create a ZedBlock
        for label in np.unique(labels):
            if label == -1:
                continue  # Skip noise points

            block = ZedBlock(cluster_points=data[labels == label])
            if PLOTH_GRAPHS:
                block.plot()
            self.zed_blocks.append(block)

        # Sort the blocks detected via pointCloud for "accuracy", aka "how far away from 90° the angle between v1 and v2 is"
        self.zed_blocks = sorted(self.zed_blocks, key=lambda block: block.accuracy)
        
        if PLOTH_GRAPHS:
            plt.xlabel('x')
            plt.ylabel('y')
            plt.axis('equal')
            plt.gca().set_xlim([0,1.0])
            plt.gca().set_ylim([0.15,0.8])
            #plt.gca().set_clip_on(True)
            plt.gca().invert_yaxis()
            plt.gca().invert_xaxis()
            plt.savefig(f'{pointCloud_path}/2D_blocks.png')
        
        end_time = time.time()
        print(f"< POINT CLOUD PROCESS: ENDED after {end_time-start_time}")
        

        # waiting for object detection's thread
        threadObjDetection.join()


        # ##################### BBOX IN GRAFICO 2D #######################

        # per ciascun oggetto riconosciuto dalla rete neurale, riportiamo la bbox nel mondo 2d (visto da sopra)
        # valutiamo quindi se il centro dell'oggetto riconosciuto tramite point cloud sta in una sola bbox o più
        # se sta in una sola bbox, allora siamo sicuri di quale oggetto è stato riconosciuto da Yolo
        # se il centro sta in più di una bbox, allora salto il blocco nella speranza che, rimuovendo blocchi nel mentre, la situazione si sistemi

        from scipy.spatial import ConvexHull, Delaunay

        class YoloDetectionObj:
            """!
            Class to save the predictions and the information returned from the model prediction.
            """
            def __init__ (self, obj_class: int, confidence: float, delaunay: Delaunay, id_box):
                ## Object class detected by the model
                self.obj_class = obj_class
                ## Confidence detected by the model
                self.confidence = confidence
                ## Delaunay object to represent the bbox
                self.delaunay = delaunay
                ## ID bbox
                self.id_box = id_box
                
        for prediction in self.predicted_objects:
            x1 = math.floor(prediction[0])
            y1 = math.floor(prediction[1]) 
            x2 = math.floor(prediction[2])
            y2 = math.floor(prediction[3]) 
            confidence = prediction[4]
            #obj_class = int(prediction[5])
            obj_name = prediction[6]

            # Project the borders of bbox in point cloud coordinates, and in (2D) world ones
            borders = []
            borders.append((x1,y1))
            borders.append((x1,y2))
            borders.append((x2,y1))
            borders.append((x2,y2))

            borders_world = self.manage_cloud.extract_point_world(borders, dim=2)
            borders_world = np.array(borders_world)

            # Create a Delaunay object to better manage the bbox area
            delaunay = Delaunay(borders_world)
            block_detected = YoloDetectionObj(obj_name, confidence, delaunay, self.predicted_objects.index(prediction)+1)
            self.yolo_blocks.append(block_detected)
            
            if PLOTH_GRAPHS:
                # Create a ConvexHull object to better represent the bbox area 
                hull = ConvexHull(borders_world)

                # Plot the convex hull
                for simplex in hull.simplices:
                    plt.plot(borders_world[simplex, 0], borders_world[simplex, 1], '-', color='grey', alpha=0.3)

                # Calculate the centroid of the Convex Hull to print the object number inside
                hull_points = borders_world[hull.vertices]
                centroid = np.mean(hull_points, axis=0)
                plt.text(centroid[0], centroid[1], self.predicted_objects.index(prediction)+1, fontsize=12, ha='center', va='center', color='grey', alpha=0.3)

        if PLOTH_GRAPHS:
            plt.savefig(f'{pointCloud_path}/2D_bboxes.png')

        # Merge the info taken from the model and from the Point Cloud
        self.choose_good_objects()
        if DEBUG:
            print("Take:")
            for block in self.blocks_to_take:
                print(f"-> OBJECT {block.yolo_bbox_id + 1}\
                    \n   x: {block.mid[0]}\
                    \n   y: {block.mid[1]}\
                    \n   z: {block.mid[2]}\
                    \n   yaw: {block.yaw}\
                    \n   id: {block.yolo_prediction}\
                    ")

    def object_detection(self, imgName: str, image: Image):
        """!
        Manages the object detection process. This is threaded during the execution of the vision module.

        @param imgName: Name of the directory in which save the visual representation of the predictions.
        @param image: File of the image on which perform the prediction

        @return Nothing, it changes the object variables.
        """

        # convert received image (bgr8 format) to a cv2 image
        image_cv2 = CvBridge().imgmsg_to_cv2(image, "bgr8")
        cv2.imwrite(f'camera-rolls/{imgName}.png', image_cv2)

        # ##################### OBJECT DETECTION #######################
        
        print("> OBJECT DETECTION PROCESS: STARTED")
        start_time = time.time()

        # predict with the object-detection model
        prediction_path = f"predictions/{imgName}/yoloV8"
        ## List of predicted objects from the YoloV8 model
        self.predicted_objects = self.predictor.predict(image=image_cv2, path_to_save_prediction=prediction_path, top_crop=370, bottom_crop=130, print_to_console=False)

        end_time = time.time()
        print(f"< OBJECT DETECTION PROCESS: ENDED after {end_time-start_time}")

    def choose_good_objects(self):
        """!
        Algorithm that allows to merge the info obtained via the object detection made by the model and the point cloud info. 
        It also chooses which blocks are "correctly detected", the ones that will be returned to the manipulator.

        @return Nothing, it changes the object variables.
        """

        for block in self.zed_blocks:
            
            discard = False
            
            for bbox in self.yolo_blocks:
                if bbox.delaunay.find_simplex(block.mid) >= 0:
                    if discard:     # The block is inside multiple bounding boxes
                        if DEBUG:
                            print('  -> Il blocco è stato scartato. Troppe bbox.')
                            print()
                            print(block)
                            print("-----------------------------")
                        block.yolo_bbox_id = None
                        break
                    else:
                        if DEBUG:
                            print("  -> Prima bbox trovata")
                        discard = True
                        block.yolo_bbox_id = self.yolo_blocks.index(bbox)
            
            if block.yolo_bbox_id is not None:

                if DEBUG:
                    print("  -> Blocco con bbox")
                
                # Copy all the info in the ZedBlock object
                block.yolo_confidence = self.yolo_blocks[block.yolo_bbox_id].confidence
                block.yolo_prediction = self.yolo_blocks[block.yolo_bbox_id].obj_class

                if block.yolo_confidence > 0.65:
                    if DEBUG:
                        print("  -> il blocco ha confidence >0.65")
                    block.compute_mid_3d()
                    v1_norm = round(block.n1 / UNIT_LENGTH,0)
                    v1_norm += (block.n1 / UNIT_LENGTH - v1_norm > NORM_TRESHOLD)
                    v2_norm = round(block.n2 / UNIT_LENGTH,0)
                    v2_norm += (block.n2 / UNIT_LENGTH - v2_norm > NORM_TRESHOLD)

                    expected_sizes = Models[block.yolo_prediction].factor
                    expected_max_size = max(expected_sizes.width, expected_sizes.length)
                    expected_min_size = min(expected_sizes.width, expected_sizes.length)
                    
                    if v1_norm == expected_max_size and v2_norm == expected_min_size:
                        if DEBUG:
                            print(f"  <- BLOCCO PRESO con v1_norm={v1_norm}, v2_norm={v2_norm}")
                        self.blocks_to_take.append(block)

                if DEBUG:
                    print()
                    print(block)
                    print("-----------------------------")

        first_with_box = -1
        if len(self.blocks_to_take) == 0:
            # the idea is: take only one block, the one that respects some pre-defined requisites
            # and is the closer to the zed camera (the vertex has a smaller x as possible)

            if DEBUG:
                print("  -> Lista blocchi da prendere vuota")

            # Order the blocks for x closer to 0
            self.zed_blocks = sorted(self.zed_blocks, key=lambda block: block.vertex[0])
            for block in self.zed_blocks:
                if block.yolo_bbox_id is not None:
                    if DEBUG:
                        print("  -> Blocco con box")
                    taken = False
                    if first_with_box<0 :
                        first_with_box = self.zed_blocks.index(block)

                    if block.n2 == 0:
                        if DEBUG:
                            print("  -> Un solo lato")

                        # The ZedCamera sees only one side, therefore we have to see whether this side matches
                        # one of the sides predicted by the model

                        v1_norm = round(block.n1 / UNIT_LENGTH,0)
                        v1_norm += (block.n1 / UNIT_LENGTH - v1_norm > NORM_TRESHOLD)

                        expected_sizes = Models[block.yolo_prediction].factor
                        expected_max_size = max(expected_sizes.width, expected_sizes.length)
                        expected_min_size = min(expected_sizes.width, expected_sizes.length)

                        if v1_norm  == expected_max_size:
                            if DEBUG:
                                print("  -> Il lato è quello big")
                            block.p2 = np.array([block.vertex[0] + expected_min_size*UNIT_LENGTH/2, block.vertex[1]])
                            block.v2 = block.p2 - block.vertex
                            block.n2 = round(np.linalg.norm(block.v2),3)
                            block.compute_mid()
                            block.compute_mid_3d()
                            block.compute_yaw()

                            taken = True

                        elif v1_norm == expected_min_size:
                            if DEBUG:
                                print("  -> Il lato è quello small")
                            block.p2 = np.array([block.vertex[0] + expected_max_size*UNIT_LENGTH/2, block.vertex[1]])
                            block.v2 = block.p2 - block.vertex
                            block.n2 = round(np.linalg.norm(block.v2),3)

                            block.p1, block.p2 = exchange(block.p1, block.p2)
                            block.v1, block.v2 = exchange(block.v1, block.v2)
                            block.n1, block.n2 = exchange(block.n1, block.n2)
                            block.compute_mid()
                            block.compute_mid_3d()
                            block.compute_yaw()

                            taken = True

                    else:

                        if DEBUG:
                            print("  -> Due lati")

                        # If the block is not a one-detected-side one, then we could check if the model
                        # predicted the right sides' length

                        v1_norm = round(block.n1 / UNIT_LENGTH,0)
                        v1_norm += (block.n1 / UNIT_LENGTH - v1_norm > NORM_TRESHOLD)
                        v2_norm = round(block.n2 / UNIT_LENGTH,0)
                        v2_norm += (block.n2 / UNIT_LENGTH - v2_norm > NORM_TRESHOLD)

                        expected_sizes = Models[block.yolo_prediction].factor
                        expected_max_size = max(expected_sizes.width, expected_sizes.length)
                        expected_min_size = min(expected_sizes.width, expected_sizes.length)
                        
                        if v1_norm == expected_max_size and v2_norm == expected_min_size:
                            if DEBUG:
                                print("  -> Lati riconosciuti correttamente [quindi confidence < 0.65]")
                            block.compute_mid_3d()
                            taken = True
                        else:
                            if DEBUG:
                                print(f"  <- Lati SBAGLIATI con v1_norm={v1_norm}, v2_norm={v2_norm}")

                    # If the object is taken, just pass this one
                    if taken:
                        if DEBUG:
                            print("  <- BLOCCO PRESO")
                            print()
                            print(block)
                            print("-----------------------------")

                        self.blocks_to_take.append(block)
                        return
                    
            # If nothing better was found, we return the block closer to the camera which was identified
            # from the model, using the z-value from the model
            if first_with_box != -1 :
                if DEBUG:
                    print("  -> Nothing better was found. Block with bbox")
                    print("  -> We will use the x and y values from the point cloud, z (and prediction) from the model.")
                    print("  <- BLOCCO PRESO")

                block = self.zed_blocks[first_with_box]
                
                # change yolo prediction
                height_factor = float(re.findall(r'[X,Y,Z][0-9]', block.yolo_prediction)[2][1:])
                last_factor = block.yolo_prediction.split('-')[-1]
                is_special = last_factor[-1].isalpha()

                new_prediction = f"X{int(v2_norm)}-Y{int(v1_norm)}-Z{int(height_factor)}"
                if is_special:
                    special_prediction = f"{new_prediction}-{last_factor}"
                    try:
                        if Models[special_prediction] is not None:
                            new_prediction = special_prediction
                    except KeyError:
                        pass
                try:
                    if Models[new_prediction] is not None:
                        block.yolo_prediction = new_prediction
                except KeyError:
                    pass
                ######

                block.compute_mid_3d()
                block.compute_yaw()

                self.blocks_to_take.append(block)

                if DEBUG:
                    print(block)
                    print("-----------------------------")
            else:
                if DEBUG:
                    print("  -> Nothing better was found. Block WITHOUT bbox")
                # We still have one or more block on the working-table, these haven't been detected by the model
                block = self.zed_blocks[0]
                if block.v2 is not None:

                    v1_norm = round(block.n1 / UNIT_LENGTH,0)
                    v1_norm += (block.n1 / UNIT_LENGTH - v1_norm > NORM_TRESHOLD)
                    v2_norm = round(block.n2 / UNIT_LENGTH,0)
                    v2_norm += (block.n2 / UNIT_LENGTH - v2_norm > NORM_TRESHOLD)

                    v1_norm = max(int(v1_norm), 1)
                    v2_norm = max(int(v2_norm), 1)
                    block.yolo_prediction = f"X{v2_norm}-Y{v1_norm}-Z2"
                    block.yolo_bbox_id = -1
                    block.compute_mid_3d()
                    self.blocks_to_take.append(block)

                    if DEBUG:
                        print("  -> We will use the x and y values from the point cloud, z=2. con v1_norm={v1_norm}, v2_norm={v2_norm}")
                        print("  <- BLOCCO PRESO")
                        print()
                        print(block)
                        print("-----------------------------")
                else:
                    if DEBUG:
                        print("  -> Can't do anything else.")


        return

    def handle_get_blocks(self):
        """!
        This function gets called in order to reply to the service's calls coming from the clients.
        
        @return Nothing
        """

        print('VISION PROCESS: handle_obtain_blocks CALLED')

        poses = []
        blocks_id = []
        n_blocks = len(self.blocks_to_take)
        finished = self.has_finished_blocks

        print(f'The blocks to be SENT are {n_blocks}, the following lines describe them')

        for block in self.blocks_to_take:
            print(f'{block.yolo_prediction} centered in ({round(block.mid[0], 3)}, {round(block.mid[1], 3)}, {round(block.mid[2], 3)})')

            pose = Pose()

            # position
            pose.position.x = block.mid[0]
            pose.position.y = block.mid[1]
            pose.position.z = block.mid[2]
            
            # quaternion
            pose.orientation.w = math.cos(block.yaw/2)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = math.sin(block.yaw/2)

            poses.append(pose)
            blocks_id.append(block.yolo_prediction)

        print('VISION PROCESS: handle_obtain_blocks ENDED')

        return GetBlocksResponse(poses, blocks_id, np.int8(n_blocks), finished)

class RosManager():
    """!
    This class manages the interaction between the ROS clients and the vision module,
    calling continously the lattest each time a new request is done.
    """

    def __init__(self):
        # Service's definition and its handler's setting
        ros.init_node('vision')

    def start_service(self):
        """!
        Starts the Vision's ROS service. The service should be called from the client and it recalls the function "callback".

        @return Nothing
        """
        s = ros.Service('vision', GetBlocks, self.callback)
        print('VISION Process: Service STARTED')

        # The node runs indefinitely
        ros.spin()

    def callback(self, req):
        """!
        This function recalls the VisionManagerClass, so each time a client makes a request the vision module computes the table data from zero.

        @return Nothing
        """
        manager = VisionManagerClass()
        return manager.handle_get_blocks()

def main():
    """!
    Starts a ROS Node and keeps it running.
    """
    print('VISION PROCESS: STARTED')

    # check program's arguments
    DEBUG="debug" in sys.argv[1:]
    test="test" in sys.argv[1:]

    manager = RosManager()
    if test:
        manager.callback(None)
    else:
        manager.start_service()


if __name__ == '__main__':
    main()