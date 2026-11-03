#!/usr/bin/env python3

import rclpy
import os
import sys

from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from tf_transformations import quaternion_from_euler

def pack_PoseMsg(x:float, y:float, theta:float) -> Pose:
    msg = Pose()

    msg.position.x = float(x)
    msg.position.y = float(y)
    msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w = quaternion_from_euler(0,0,theta)

    return msg

class Loader( Node ):
    def __init__( self, id:str, topic: str = "goal_list"):
        super().__init__( id )
        self.poses_publisher = self.create_publisher(PoseArray, topic, 2)

    def publish_from_list( self, poses: list[tuple]):
        msg = PoseArray()

        for pose_tuple in poses:
            pose = pack_PoseMsg(*pose_tuple)
            msg.poses.append(pose)

        self.poses_publisher.publish(msg)
        self.get_logger().info(
            f"[Sent] [PoseArray of {len(poses)}]")     

    def read_poses_from_file( self, filepath: os.PathLike ):
        """
        Recibe un filepath y retorna una lista de tuplas de poses
        """
        with open(filepath, "r") as file:
            lines: list[str] = file.readlines()

        poses_list = []
        for line in lines:
            data = line.strip()[1:-1].split(",")
            poses_list.append((float(data[0]), float(data[1]), float(data[2])))
        
        return poses_list

    def publish_from_file(self, filepath: os.PathLike):
        self.publish_from_list(self.read_poses_from_file(filepath))
    
def main ( args = None):
    rclpy.init( args = args )

    if len(sys.argv) > 1:
        poses_filepath = os.path.join("src","lab2","routes",sys.argv[1])
    else:
        poses_filepath = os.path.join("src","lab2","routes","poses.txt")

    publisher = Loader("pose_loader")
    publisher.publish_from_file(poses_filepath)

    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
