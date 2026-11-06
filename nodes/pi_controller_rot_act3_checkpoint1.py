#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

# Parametros
IGNORE_VALUE = 100000.0

class PIDController( Node ):

    def __init__( self, kp, ki = 0, kd = 0 ):
        super().__init__( 'pi_controller_rot' )
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = 0.0
        self.state = None
        self.prev_state = None
        self.prev_time = time.time()
        self.proportional_action = 0
        self.accumulated_error = 0

        self.actuation_pub = self.create_publisher( Twist, '/commands/velocity', 1 )
        # self.dist_set_point_sub = self.create_subscription( Float64, 'setpoint_rotational', self.setpoint_cb, 1 )
        self.rot_state_sub = self.create_subscription( Float64, 'rot_state', self.state_cb, 1 )

    # def setpoint_cb( self, msg ):
    #   self.get_logger().info( '[PICTRL] new setpoint received: %.2f' % (msg.data) )
    #   self.reset()
    #   self.setpoint = msg.data

    def state_cb( self, msg ):
        self.prev_state = self.state
        self.state = msg.data
        self.get_logger().info( f"angular state value: {self.state}" ) 
        if self.state == (None or IGNORE_VALUE):
            self.get_logger().info( f"entered if" ) 
            return
        error = self.setpoint - self.state
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time
        
        # Proportional
        p_actuation = self.kp*error

        # Integrative 
        self.accumulated_error += ( error * dt )
        i_actuation = self.ki*(self.accumulated_error)

        # Derivative 
        # prev_error = ( self.setpoint ) if ( self.prev_state == None ) else ( self.setpoint - self.prev_state )    
        # d_actuation = self.kd*( error - prev_error ) / dt
        #                (error_{k+1} - error_{k}) / time_delta

        # Actuation
        actuation = p_actuation + i_actuation #+ d_actuation

        # Message sending
        msg = Twist()
        msg.angular.z = actuation
        self.actuation_pub.publish( msg )

        self.get_logger().info( f"linear actuation OFF" ) 
        self.get_logger().info( f"rotary actuation: {actuation}" )

    def reset( self ):
        self.setpoint = None
        self.state = None


def main():
    rclpy.init()
    p_ctrl = PIDController( 0.1, 0.05, 0 )
    rclpy.spin( p_ctrl )

if __name__ == '__main__':
    main()



