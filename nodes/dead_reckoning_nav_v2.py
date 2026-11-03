#!/usr/bin/env python3

import rclpy
import os, time, tempfile
import numpy as np
from typing import NamedTuple
from threading import Thread, Timer
import matplotlib.pyplot as plt

from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Pose as PoseMsg
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf_transformations import euler_from_quaternion

# Parametros
LINEAR_SPEED = 0.2   # m/s
ANGULAR_SPEED = 1.0     # rad/s

STOP_MSG = Twist()
STOP_MSG.linear.x = 0.0
STOP_MSG.angular.z = 0.0

class SpeedCommand(NamedTuple):
    linear: float
    angular: float
    time: float

class RegistroPosicion(NamedTuple):
    timestamp: float
    x: float
    y: float
    theta: float

class Displacement(NamedTuple):
    linear_disp: float
    rotational_disp: float

class Pose(NamedTuple):
    '''
    Convenciones:
    - x positivo hacia la derecha
    - y positivo hacia arriba
    - theta=0 hacia la derecha, giro contrarreloj, medido en radianes
    '''
    x: float
    y: float
    theta: float

def unpack_PoseMsg(msg: PoseMsg) -> Pose:
    q = [
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    ]
    yaw = euler_from_quaternion(q)[2]

    return Pose(
        x=float(msg.position.x),
        y=float(msg.position.y),
        theta=yaw
    )

class DeadReckoningNavigator(Node):
    def __init__(self, 
                 id:str,
                 vel_topic: str = "/cmd_vel",
                 real_pose_topic: str = "/real_pose",
                 odom_topic: str = "/odom",
                 goal_list_topic: str = "/goal_list",
                 pos_topic: str = "/setpoint_linear",
                 rot_topic: str = "/setpoint_rotational",
                 pos_state_topic: str = "/pos_state",
                 rot_state_topic: str = "/rot_state",
                 vel_control_topic: str = "/control_effort",
                 rot_control_topic: str = "/rot_control_effort",):
        super().__init__(id)

        self.subscription_load = self.create_subscription(PoseArray, goal_list_topic, self.accion_mover_cb, 2)
        self.subscription_check_pose = self.create_subscription(PoseMsg, real_pose_topic, self.read_real_pose, 1)
        self.subscription_odom = self.create_subscription(Odometry, odom_topic, self.read_odom, 2)
        self.subscription_controller_vel = self.create_subscription(Float64, vel_control_topic, self.read_control_vel, 2)
        self.subscription_controller_rot = self.create_subscription(Float64, rot_control_topic, self.read_control_rot, 2)
        self.vel_publisher = self.create_publisher(Twist, vel_topic, 2)
        self.pos_publisher = self.create_publisher(Float64, pos_topic, 2)
        self.rot_publisher = self.create_publisher(Float64, rot_topic, 2)
        self.pos_state_publisher = self.create_publisher(Float64, pos_state_topic, 2)
        self.rot_state_publisher = self.create_publisher(Float64, rot_state_topic, 2)

        self.threads = []
        self.real_pose = Pose(1,1,0)
        self.linear_vel = 0.0
        self.rotational_vel = 0.0

        self.odom_pos_data: list[RegistroPosicion] = []
        self.real_pos_data: list[RegistroPosicion] = []

    def aplicar_velocidad(self, position_commands: list, final_stop: bool = True, blocking: bool = False):
        '''
        pending...
        '''

        # SECCION LAB 2

        for index in range(len(position_commands)):
            # Crear mensaje
            linear = True if position_commands[index].linear_disp != 0.0 else False
            msg_obj = Float64()
            msg_state = Float64()

            self.get_logger().info(f"[Sending] [Linear: {position_commands[index].linear_disp}, Angular: {position_commands[index].rotational_disp}]")
            run = True
            if linear:
                msg_obj.data = position_commands[index].linear_disp
                self.pos_publisher.publish(msg_obj)
                if index == 0:
                    while run:
                        msg_state.data = float(self.real_pos_data[-1].x)
                        self.pos_state_publisher.publish(msg_state)
                        if self.real_pos_data[-1].x == position_commands[index].linear_disp:
                            run = False
                        msg_vel = Twist()
                        msg_vel.linear.x = self.linear_vel
                        self.get_logger().info(f"[Sending] []")
                        self.vel_publisher.publish(msg_vel)

                elif index == 2:
                    while run:
                        msg_state.data = float(self.real_pos_data.y[-1])
                        self.pos_state_publisher.publish(msg_state)
                        if self.real_pos_data[-1].y == position_commands[index].linear_disp:
                            run = False
                        msg_vel = Twist()
                        msg_vel.linear.x = self.linear_vel
                        self.vel_publisher.publish(msg_vel)
                
            else:
                msg_obj.data = position_commands[index].rotational_disp
                self.rot_publisher.publish(msg_obj)
                while run:
                    msg_state.data = self.real_pos_data[-1].theta
                    self.rot_state_publisher.publish(msg_state.data)
                    if self.real_pos_data[-1].theta == position_commands[index].rotational_disp:
                        run = False
                    msg_vel = Twist()
                    msg_vel.angular.z = self.rotational_vel
                    self.vel_publisher.publish(msg_vel)
            self.pos_publisher.publish(None)
            self.rot_publisher.publish(None)
            self.vel_publisher.publish(None)


        # SECCION LAB1

        # def thread_exc(thread_to_join: Thread):
        #     if thread_to_join:
        #         thread_to_join.join()
        #         self.threads.remove(thread_to_join)

        #     for speed_command in speed_commands:
        #         # Crear mensaje
        #         msg = Twist()
        #         msg.linear.x = speed_command.linear
        #         msg.angular.z = speed_command.angular
        #         target_time = speed_command.time

        #         # Publicar mensaje
        #         sent_time = time.time()
        #         self.get_logger().info(f"[Sending] [Linear: {speed_command.linear}, Angular: {speed_command.angular}]")
                
        #         while time.time() - sent_time < target_time: 
        #             self.vel_publisher.publish(msg)
        #             #self.get_logger().info(
        #             #    f"[Sent] [Linear: {msg.linear.x}, Angular: {msg.angular.z}] Remaining Time:{target_time- (time.time()- sent_time)}")
        #             #time.sleep(0.01)

        #     # Detener al final
        #     if final_stop:
        #         self.vel_publisher.publish(STOP_MSG)
        #         self.get_logger().info(
        #             f"POSE COMPLETE - STOPPING [Sent] [Linear: {STOP_MSG.linear.x}, Angular: {STOP_MSG.angular.z}]")

        # if not blocking:
        #     thread = Thread(target=thread_exc, args=(self.threads[-1] if self.threads else None,), daemon=True)
        #     self.threads.append(thread)
        #     thread.start()
        # else:
        #     thread_exc(None)

    def get_speed_commands(self, goal_pose: Pose, init_pose: Pose = Pose(0, 0, 0), factor_correccion: float = 1.0) -> list[SpeedCommand]:
        '''
        Retorna la secuencia de comandos de velocidad (y su tiempo) para llegar a goal_pose desde (0,0,0).
        Multiplica los tiempos de giro por `factor_correccion`.
        '''
        if not isinstance(goal_pose, Pose):
            goal_pose = Pose(goal_pose[0], goal_pose[1], goal_pose[2])
        speed_commands: list[SpeedCommand] = []

        if goal_pose.x != init_pose.x:
            # Orientar hacia +-X
            theta_to_x = 0 if goal_pose.x > init_pose.x else np.pi
            speed_commands.append(SpeedCommand(
                linear=0.0,
                angular=ANGULAR_SPEED * (1 if theta_to_x > init_pose.theta else -1),
                time=abs(theta_to_x - init_pose.theta) * factor_correccion / ANGULAR_SPEED
            ))

            # Movimiento en X
            speed_commands.append(SpeedCommand(
                linear=LINEAR_SPEED,
                angular=0.0,
                time=abs(goal_pose.x - init_pose.x) / LINEAR_SPEED
            ))
        else:
            theta_to_x = init_pose.theta

        if goal_pose.y != init_pose.y:
            # Orientar hacia +Y
            theta_to_y = np.pi/2 if goal_pose.y > init_pose.y else -np.pi/2
            speed_commands.append(SpeedCommand(
                linear=0.0,
                angular=ANGULAR_SPEED * (1 if theta_to_y > theta_to_x else -1),
                time=abs(theta_to_y - theta_to_x) * factor_correccion / ANGULAR_SPEED
            ))

            # Movimiento en Y
            speed_commands.append(SpeedCommand(
                linear=LINEAR_SPEED,
                angular=0.0,
                time=abs(goal_pose.y - init_pose.y) / LINEAR_SPEED
            ))
        else:
            theta_to_y = theta_to_x

        # Orientar hacia theta objetivo
        speed_commands.append(SpeedCommand(
            linear=0.0,
            angular=ANGULAR_SPEED * (1 if goal_pose.theta > theta_to_y else -1),
            time=abs(goal_pose.theta - theta_to_y) * factor_correccion / ANGULAR_SPEED
        ))

        return [speed_command for speed_command in speed_commands if speed_command.time > 0]

    def mover_robot_a_destino(self, goal_pose: Pose, init_pose: Pose = Pose(0, 0, 0), blocking = False):
        '''
        pending...
        '''
        displacement_0 = Displacement(linear_disp=float(goal_pose[0]), rotational_disp=0.0)
        displacement_1 = Displacement(linear_disp=0.0, rotational_disp=float(goal_pose[2]))
        displacement_2 = Displacement(linear_disp=float(goal_pose[1]), rotational_disp=0.0)

        displacement_list = [displacement_0, displacement_1, displacement_2]

        self.get_logger().info(f"displacement_0: {displacement_0}\ndisplacement_1: {displacement_1}\ndisplacement_2: {displacement_2}\ndisplacement_list: {displacement_list}")

        self.aplicar_velocidad(displacement_list, blocking= blocking)

    def save_data(self, thread_to_join: Thread = None):
        if thread_to_join:
            thread_to_join.join()
            self.threads.remove(thread_to_join)

        dir_path = os.path.join("install", "lab1", "data",f"pos_data_{round(time.time())}")
        os.makedirs(dir_path, exist_ok=True)

        with open(os.path.join(dir_path,"odom.csv"), "w") as f:
            f.writelines((",".join(map(lambda x: str(x), registro)) + "\n" for registro in self.odom_pos_data))

        with open(os.path.join(dir_path,"real.csv"), "w") as f:
            f.writelines((",".join(map(lambda x: str(x), registro)) + "\n" for registro in self.real_pos_data))

        # Gráfico
        DIM_X, DIM_Y = 5.5 , 2.9 # m
        odom_x, odom_y = [x for t,x,y in self.odom_pos_data],[y for t,x,y in self.odom_pos_data]
        real_x, real_y = [x for t,x,y in self.real_pos_data],[y for t,x,y in self.real_pos_data]
        self.odom_pos_data = []
        self.real_pos_data = []

        fig, ax = plt.subplots()
        ax.plot(odom_x, odom_y, label="Odometría")
        ax.plot(real_x, real_y, label="Pose Real")
        ax.set_title("Odometría vs. Pose Real")
        ax.legend()
        ax.set_xlim(0, DIM_X)
        ax.set_ylim(0, DIM_Y)
        ax.set_aspect("equal", adjustable="box")
        fig.savefig(os.path.join(dir_path, "figura_1.png"), bbox_inches="tight")
        plt.close(fig)

        return fig, ax
        
    # Suscripciones
    def accion_mover_cb(self, desired_poses_msg: PoseArray):
        self.get_logger().info(f"[Recived] [PoseArray of lengh {len(desired_poses_msg.poses)}]")
        poses = map(lambda msg: unpack_PoseMsg(msg), desired_poses_msg.poses)

        last_pose = self.real_pose
        for pose in poses:
            self.mover_robot_a_destino(goal_pose=pose, init_pose=last_pose)
            last_pose = pose

        th = Thread(target= self.save_data, args=( self.threads[-1] if self.threads else None,))
        th.start()

    def read_real_pose(self, msg: PoseMsg):
        self.real_pose = unpack_PoseMsg(msg)
        self.real_pos_data.append(RegistroPosicion(time.time(), self.real_pose.x, self.real_pose.y, self.real_pose.theta))

    def read_odom(self, msg: Odometry):
        pose = unpack_PoseMsg(msg.pose.pose)
        self.odom_pos_data.append(RegistroPosicion(time.time(), pose.x, pose.y, pose.theta))

    def read_control_vel(self, msg: Float64):
        self.linear_vel = msg

    def read_control_rot(self, msg: Float64):
        self.rotational_vel = msg

def main(args=None):
    rclpy.init(args=args)
    
    navegador  = DeadReckoningNavigator("dead_reckoning_nav")
    rclpy.spin(navegador)

    navegador.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# ------ DUDAS ------ #
# Rango de angulos en twist
# Pueden pedirme coordenadas negativas?
