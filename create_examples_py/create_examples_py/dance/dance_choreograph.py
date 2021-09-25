import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import LedColor
from irobot_create_msgs.msg import LightringLeds

class ColorPalette():
    """ Helper Class to define frequently used colors"""
    def __init__(self):
        self.red = LedColor(red=255,green=0,blue=0)
        self.green = LedColor(red=0,green=255,blue=0)
        self.blue = LedColor(red=0,green=0,blue=255)
        self.yellow = LedColor(red=255,green=255,blue=0)
        self.pink = LedColor(red=255,green=0,blue=255)
        self.cyan = LedColor(red=0,green=255,blue=255)
        self.purple = LedColor(red=127,green=0,blue=255)
        self.white = LedColor(red=255,green=255,blue=255)
        self.grey = LedColor(red=189,green=189,blue=189)

class Move():
    """ Class to tell the robot to move as part of dance sequence"""
    def __init__(self, x_m_s, theta_degrees_second):
        """
        Parameters
        ----------
        x_m_s : float
            The speed to drive the robot forward (positive) /backwards (negative) in m/s    
        theta_degrees_second : float
            The speed to rotate the robot counter clockwise (positive) / clockwise (negative) in deg/s
        """
        self.x = x_m_s
        self.theta = math.radians(theta_degrees_second)

class Lights():
    """ Class to tell the robot to set lightring lights as part of dance sequence"""
    def __init__(self, led_colors):
        """
        Parameters
        ----------
        led_colors : list of LedColor
            The list of 6 LedColors corresponding to the 6 LED lights on the lightring
        """
        self.led_colors = led_colors

class FinishedDance():
    """ Class to tell the robot dance sequence has finished"""
    pass

class DanceChoreographer():
    """ Class to manage a dance sequence, returning current actions to perform"""
    def __init__(self, dance_sequence):
        '''
        Parameters
        ----------
        dance_sequence : list of (time, action) pairs
            The time is time since start_dance was called to initiate action,
            the action is one of the classes above [Move,Lights,FinishedDance]
        '''    
        self.dance_sequence = dance_sequence
        self.action_index = 0

    def start_dance(self, time):
        '''
        Parameters
        ----------
        time : rclpy::Time
            The ROS 2 time to mark the start of the sequence
        '''    
        self.start_time = time
        self.action_index = 0

    def get_next_actions(self, time):
        '''
        Parameters
        ----------
        time : rclpy::Time
            The ROS 2 time to compare against start time to give actions that should be applied given how much time sequence has been running for
        '''    
        time_into_dance = time - self.start_time
        time_into_dance_seconds = time_into_dance.nanoseconds / float(1e9)
        actions = []
        while self.action_index < len(self.dance_sequence) and time_into_dance_seconds >= self.dance_sequence[self.action_index][0]:
            actions.append(self.dance_sequence[self.action_index][1])
            self.action_index += 1
        return actions

class DanceCommandPublisher(Node):
    """ Class to publish actions produced by the DanceChoreographer"""
    def __init__(self, dance_choreographer):
        '''
        Parameters
        ----------
        dance_choreographer : DanceChoreographer
            The configured DanceChoreographer to give time and query for actions to publish
        '''    
        super().__init__('dance_command_publisher')
        self.dance_choreographer = dance_choreographer
        self.lights_publisher_ = self.create_publisher(LightringLeds, 'cmd_lightring', 10)
        self.vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.dance_choreographer.start_dance(self.get_clock().now())
        self.last_twist = Twist()
        self.last_leds = LightringLeds()
        self.last_leds.override_system = False

    def timer_callback(self):
        next_actions = self.dance_choreographer.get_next_actions(self.get_clock().now())
        twist = self.last_twist
        leds = self.last_leds
        for next_action in next_actions:
            if isinstance(next_action, Move):
                twist = Twist()
                twist.linear.x = next_action.x
                twist.angular.z = next_action.theta
                self.last_twist = twist
                self.get_logger().info('New move action: %f, %f' % (twist.linear.x, twist.angular.z))
            elif isinstance(next_action, Lights):
                leds = LightringLeds()
                leds.override_system = True
                leds.leds = next_action.led_colors
                self.last_leds = leds
                self.get_logger().info('New lights action')
            else:
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.last_twist = twist
                leds = LightringLeds()
                leds.override_system = False
                self.last_leds = leds
                self.get_logger().info('Finished Dance Sequence')

        self.vel_publisher_.publish(twist)
        self.lights_publisher_.publish(leds)
