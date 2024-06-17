#! /usr/bin/env python
import rospy as ros

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from robotics_project.srv import GetBlocks

from datetime import datetime

# una volta effettuate delle modifiche non è necessario buildare nuovamente
# il progetto, almeno che non si debbano cambiare gli import


class VisionManagerClass():

    def __init__(self, robot_name="ur5"):
        ros.init_node('vision')
        
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

        cv2.imwrite(f'camera-rolls/{imgName}.png', image_cv2)


        return
        points_list = []
        #this is the center of the image plane
        center_x = int(pc2.width / 2)
        center_y = int(pc2.height / 2)

        for data in point_cloud2.read_points(pc2, field_names=['x','y','z'], skip_nans=False, uvs=[(center_x, center_y)]):
            points_list.append([data[0], data[1], data[2]])
        print("Data Optical frame: ", points_list)
        
        # pointW = self.w_R_c.dot(points_list[0]) + self.x_c + self.base_offset
        # print("Data World frame: ", pointW)


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