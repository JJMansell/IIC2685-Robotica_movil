#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

import time, math
from typing import Literal, TypeAlias
control_type: TypeAlias = Literal["lin", "ang"]

class PIDController( Node ):

  def __init__(self):
    super().__init__('pid_controller')

    self.declare_parameter('control_type', 'lin')
    control_type = self.get_parameter('control_type').value

    self.declare_parameter('kp', 0.5)
    self.declare_parameter('ki', 0.0)
    self.declare_parameter('kd', 0.0)

    self.kp = self.get_parameter('kp').value
    self.ki = self.get_parameter('ki').value
    self.kd = self.get_parameter('kd').value

    self.actuation_pub = self.create_publisher( Float64, 'control_' + control_type, 1 )
    self.dist_set_point_sub = self.create_subscription( Float64, 'setpoint_' + control_type, self.setpoint_cb, 1 )
    self.dist_state_sub = self.create_subscription( Float64, 'state_' + control_type, self.state_cb, 1 )

    self.reset()

  def setpoint_cb( self, msg: Float64 ):
    self.get_logger().info( 'New setpoint received: %.2f' % (msg.data) )
    self.reset()
    self.setpoint = msg.data

  def state_cb( self, msg: Float64 ):
    if self.setpoint == None:
      return

    if not math.isfinite(msg.data):
      self.get_logger().warning(f"Estado no finito recibido: {msg.data}; se ignora.")
      return
    
    self.state = msg.data
    error = self.setpoint - self.state
    t = time.time()
    if t - self.memory["time"][-1] <= 0:
      return

    # Actuation
    e_n1, e_n2 = self.memory["error"][-1], self.memory["error"].pop(-2)
    t_n1, t_n2 = self.memory["time"][-1], self.memory["time"].pop(-2)
    u_n1 = self.memory["control"].pop(-1)

    control = u_n1 + self.kp * (error - e_n1) + self.ki * error * (t - t_n1) + self.kd * ((error - e_n1)/(t - t_n1) - (e_n1 - e_n2)/(t_n1 - t_n2))
    
    self.memory["control"].append(control)
    self.memory["error"].append(error)
    self.memory["time"].append(t)

    # Retornar mensaje de control
    self.actuation_pub.publish( Float64(data= control) )
    #self.get_logger().info(f"PID Control: state= {msg.data} -> control= {control}")

  def reset( self ):
    self.setpoint = 0.0
    self.state = None
    self.memory: dict[Literal["control", "error", "time"],list[float]] = {
        "control": [0.0],
        "error": [0.0,0.0],
        "time": [time.time()-1,time.time()]
    }


def main():
  rclpy.init()
  pid_ctrl = PIDController()
  rclpy.spin( pid_ctrl )

if __name__ == '__main__':
  main()