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
from robotics_project.srv import GetBlocks, GetBlocksResponse

from datetime import datetime
import os

from geometry_msgs.msg import Pose

import sys
sys.path.append("src/world")
from world import Models, TABLE_HEIGHT

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
MODEL = "dependencies/robotics_project_vision/best.pt"
ERROR_ON_Y = 30             # error found empirically. The predictor translate of 25 pixel the y coordinates of the bbox

## constants for managing point cloud
BLOCK_LEVEL = 0.875         # z value for which we are sure to not intersect with the work-station/ground
ERROR_Z = 3                 # meaning min_value -> 0.001
PLOTH_GRAPHS = True

DEBUG = False


# move to repo
from sklearn.cluster import DBSCAN
# pip install numpy matplotlib scipy scikit-learn
class Manage_Point_Cloud():
    def __init__(self, rotational_matrix, zed_wrt_world):
        self.point_cloud2_msg = None

        self.rotational_matrix = rotational_matrix
        self.zed_wrt_world = zed_wrt_world
    
    def extract_point_world(self, points_2d, dim):

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
    return b,a

class ZedBlock:
    """
    Class to represent the blocks detected from the analysis of the point cloud given by the Zed Camera
    """

    def __init__(self, cluster_points=None):
        self.cluster_points = cluster_points
        
        self.vertex : np.ndarray = None        # x,y coordinates of the vertex point

        # longest side
        self.p1 : np.ndarray = None            # x,y coordinates of the point that, together with the vertex, generates the longest side 
        self.v1 : np.ndarray = None            # vector that represents the longest side
        self.n1 : float = 0                    # length (norm) of the longest side
        # shorter side
        self.p2 : np.ndarray = None            # x,y coordinates of the point that, together with the vertex, generates the shortest side 
        self.v2 : np.ndarray = None            # vector that represents the shortest side
        self.n2 : float = 0                    # length (norm) of the shortest side
        
        self.angle : float = None              # angle [degrees] between the two sides (should be around 90°)
        self.accuracy : float = None           # accuracy defined as "how far away from 90° the angle between v1 and v2 is"

        self.mid : np.ndarray = None           # x,y coordinates of the center of the block
                        
        self.yaw : float = None                # yaw [radians] of the block, aka rotation w.r.t. z axis

        # yolo parameters
        self.yolo_bbox_id = None
        self.yolo_confidence = None
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

    def compute_vertex(self):
        point = self.cluster_points[:,0].argmin()
        self.vertex = np.array(self.cluster_points[point,:])

    def compute_sides(self):
        point_max_y = self.vertex
        point_min_y = self.vertex
        for p in self.cluster_points:
            
            if p[1] > self.vertex[1]:
                # Points below the vertex
                if p[0] >= point_max_y[0]:
                    point_max_y = p
            
            elif p[1] < self.vertex[1]:
                # Points above the vertex
                if p[0] >= point_min_y[0]:
                    point_min_y = p

            # NOTE: if p[1] == vertex[1] it means that we found a point that has the same y value of the vertex.
            # This means that the block has a side perfectly parallel to the y axis

        # Get the length of the sides
        p1 = point_min_y   
        v1 = p1 - self.vertex
        n1 = round(np.linalg.norm(v1),3)
        
        p2 = point_max_y
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
        """
        Function to compute the angle between v1 and v2
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
        self.mid = (self.p1 + self.p2) / 2

    def compute_mid_3d(self):
        # 0.019 is the height of the rectangular base of a Z1-block (meaning a Z2 block is 0.019*2)
        self.mid = [self.mid[0], self.mid[1], (TABLE_HEIGHT + (0.019*Models[self.yolo_prediction].size.height)/2)]                        

    def compute_yaw(self):
        self.yaw = np.arctan2(self.v1[1],self.v1[0])
    
    def __str__(self):
        return f"ZedBlock(vertex={self.vertex}, p1={self.p1}, v1={self.v1}, n1={self.n1}, p2={self.p2}, v2={self.v2}, n2={self.n2}, angle={self.angle}, accuracy={self.accuracy}, mid={self.mid}, yaw={self.yaw}, bbox_id={self.yolo_bbox_id})"

    def plot(self):
        # Plot v1 (longest side) in red, v2 in blue
        plt.quiver(self.vertex[0], self.vertex[1], self.v1[0], self.v1[1], angles='xy', scale_units='xy', scale=1, color='r', label='V1')
        if self.v2 is not None:
            plt.quiver(self.vertex[0], self.vertex[1], self.v2[0], self.v2[1], angles='xy', scale_units='xy', scale=1, color='b', label='V2')
        if self.mid is not None:
            plt.plot(self.mid[0], self.mid[1], 'gx')

class VisionManagerClass():

    def __init__(self, robot_name="ur5"):
        ros.init_node('vision')

        self.predictor = vision.Object_Detection(model=MODEL)
        self.manage_cloud = Manage_Point_Cloud(rotational_matrix=ROTATIONAL_MATRIX, zed_wrt_world=ZED_WRT_WORLD)
        
        # Image from ZED Node
        self.image_msg = ros.wait_for_message("/ur5/zed_node/left/image_rect_color", Image)

        # Point cloud from ZED Node
        self.point_cloud2_msg = ros.wait_for_message("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2)
        
        self.zed_blocks = []
        self.yolo_blocks = []
        self.blocks_to_take = []

        # Make use of the image obtained
        self.digest_ZED_data(self.image_msg, self.point_cloud2_msg)


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



        ################ POINT CLOUD ################

        import time
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
            if round(point[1],2) > 0.15:     # avoid the higher part of the working_table
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

        if DEBUG:
            for b in self.zed_blocks:
                print(b)
        
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
        


        ###################### OBJECT DETECTION #######################
        
        print("> OBJECT DETECTION PROCESS: STARTED")
        start_time = time.time()

        # predict with the object-detection model
        prediction_path = f"predictions/{imgName}/yoloV8"
        predicted_objects = self.predictor.predict(image=image_cv2, path_to_save_prediction=prediction_path, top_crop=370, bottom_crop=130, print_to_console=False)

        end_time = time.time()
        print(f"< OBJECT DETECTION PROCESS: ENDED after {end_time-start_time}")


        ###################### BBOX IN GRAFICO 2D #######################

        # per ciascun oggetto riconosciuto dalla rete neurale, riportiamo la bbox nel mondo 2d (visto da sopra)
        # valutiamo quindi se il centro dell'oggetto riconosciuto tramite point cloud sta in una sola bbox o più
        # se sta in una sola bbox, allora siamo sicuri di quale oggetto è stato riconosciuto da Yolo
        # se il centro sta in più di una bbox, allora salto il blocco nella speranza che, rimuovendo blocchi nel mentre, la situazione si sistemi

        from scipy.spatial import ConvexHull, Delaunay

        class YoloDetectionObj:
            def __init__ (self, obj_class: int, confidence: float, delaunay: Delaunay, id_box):
                self.obj_class = obj_class
                self.confidence = confidence
                self.delaunay = delaunay
                self.id_box = id_box
                
        for prediction in predicted_objects:
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
            block_detected = YoloDetectionObj(obj_name, confidence, delaunay, predicted_objects.index(prediction)+1)
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
                plt.text(centroid[0], centroid[1], predicted_objects.index(prediction)+1, fontsize=12, ha='center', va='center', color='grey', alpha=0.3)

        if PLOTH_GRAPHS:
            plt.savefig(f'{pointCloud_path}/2D_bboxes.png')

        # Merge the info taken from the model and from the Point Cloud
        self.choose_good_objects()
        print("Take:")
        for block in self.blocks_to_take:
            print(f"-> OBJECT {block.yolo_bbox_id + 1}\
                \n   x: {block.mid[0]}\
                \n   y: {block.mid[1]}\
                \n   z: {block.mid[2]}\
                \n   yaw: {block.yaw}\
                \n   id: {block.yolo_prediction}\
                ")
    
    #TABLE_HEIGHT 0.85
    #mid[2]=0.0095*z+TABLE_HEIGHT


    def choose_good_objects(self):
        """
        Algorithm that allows to merge the info obtained via the object detection made by the model and the point cloud info.
        The blocks are chosen as follow:
        #TODO
        """

        for block in self.zed_blocks:

            if block.mid is not None:
                discard = False
                
                for bbox in self.yolo_blocks:
                    if bbox.delaunay.find_simplex(block.mid) >= 0:
                        if discard:     # The block is inside multiple bounding boxes
                            block.yolo_bbox_id = None
                            break
                        else:
                            discard = True
                            block.yolo_bbox_id = self.yolo_blocks.index(bbox)
                
                if block.yolo_bbox_id is not None:
                    
                    # Copy all the info in the ZedBlock object
                    block.yolo_confidence = self.yolo_blocks[block.yolo_bbox_id].confidence
                    block.yolo_prediction = self.yolo_blocks[block.yolo_bbox_id].obj_class

                    if block.yolo_confidence > 0.65:
                        block.compute_mid_3d()
                        v1_norm = round(block.n1 / 0.031,0)
                        v2_norm = round(block.n2 / 0.031,0)

                        expected_sizes = Models[block.yolo_prediction].factor
                        expected_max_size = max(expected_sizes.width, expected_sizes.length)
                        expected_min_size = min(expected_sizes.width, expected_sizes.length)
                        
                        if v1_norm == expected_max_size and v2_norm == expected_min_size:
                            self.blocks_to_take.append(block)

        first_with_box = -1
        if len(self.blocks_to_take) == 0:
            # the idea is: take only one block, the one that respects some pre-defined requisites
            # and is the closer to the zed camera (the vertex has a smaller x as possible)

            # Order the blocks for x closer to 0
            self.zed_blocks = sorted(self.zed_blocks, key=lambda block: block.vertex[0])
            for block in self.zed_blocks:
                if block.yolo_bbox_id is not None:
                    taken = False
                    if first_with_box<0 :
                        first_with_box = self.zed_blocks.index(block)

                    if block.n2 == 0:
                        # The ZedCamera sees only one side, therefore we have to see whether this side matches
                        # one of the sides predicted by the model

                        v1_norm = round(block.n1 / 0.031,0)

                        expected_sizes = Models[block.yolo_prediction].factor
                        expected_max_size = max(expected_sizes.width, expected_sizes.length)
                        expected_min_size = min(expected_sizes.width, expected_sizes.length)

                        if v1_norm  == expected_max_size:
                            block.p2 = np.array([block.vertex[0] + expected_min_size*0.031/2, block.vertex[1]])
                            block.v2 = block.p2 - block.vertex
                            block.n2 = round(np.linalg.norm(block.v2),3)
                            block.compute_mid()
                            block.compute_mid_3d()
                            block.compute_yaw()

                            taken = True

                        elif v1_norm == expected_min_size:
                            block.p2 = np.array([block.vertex[0] + expected_max_size*0.031/2, block.vertex[1]])
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
                        # If the block is not a one-detecteed-side one, then we could check if the model
                        # predicted the right sides' length

                        v1_norm = round(block.n1 / 0.031,0)
                        v2_norm = round(block.n2 / 0.031,0)

                        expected_sizes = Models[block.yolo_prediction].factor
                        expected_max_size = max(expected_sizes.width, expected_sizes.length)
                        expected_min_size = min(expected_sizes.width, expected_sizes.length)
                        
                        if v1_norm == expected_max_size and v2_norm == expected_min_size:
                            block.compute_mid_3d()
                            self.blocks_to_take.append(block)
                            taken = True

                    # If the object is taken, just pass this one
                    if taken:
                        self.blocks_to_take.append(block)
                        return
                    
            # If nothing better was found, we return the block closer to the camera which was identified
            # from the model, using the z-value from the model
            if first_with_box != -1 :
                block = self.zed_blocks[first_with_box]
                block.compute_mid_3d()
                self.blocks_to_take.append(block)
                print("Nothing better was found")
            else:
                # We still have one or more block on the working-table, but these are not detecteed by the model
                #TODO: is it really necessary to implement this case?
                pass

        #TODO: manca da gestire i blocchi posti in alto a destra (nel plot dei blocchi), ovvero quei blocchi che, se
        #presentano una certa angolazione, il vertex non viene posizionato nel posto giusto

        return

    def start_service(self):
        # Service's definition and its handler's setting
        s = ros.Service('vision', GetBlocks, self.handle_get_blocks)
        print('VISION Process: Service STARTED')

        # The node runs indefinitely
        ros.spin()

    def handle_get_blocks(self, req):
        """
        This function gets called in order to reply to the service's calls coming from the clients.
        Arguments:
            Nothing
        Returns:
            Nothing
        """

        print('VISION PROCESS: handle_obtain_blocks CALLED')

        poses = []
        blocks_id = []
        n_blocks = len(self.blocks_to_take)

        print(f'The blocks to be SENT are {n_blocks}, the following lines describe them')

        for block in self.blocks_to_take:
            print(f'{block.id_class} with centre in ({block.mid[0]}, {block.mid[1]}) and an angle of {block.yaw}')

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
            blocks_id.append(block.id_class)

        print('VISION PROCESS: handle_obtain_blocks ENDED')

        return GetBlocksResponse(poses, blocks_id, np.int8(n_blocks))


def main():
    print('VISION PROCESS: STARTED')

    # Starting the ROS Node and keep it running
    manager = VisionManagerClass()
    manager.start_service()


if __name__ == '__main__':
    main()