#!/usr/bin/env python
# User libs
from keyboard import RoboKeyboardControl
from speech_commands.command_handler import CommandQueue

# Python libs
import random as rand
import math

# Ros libs
import rospy

# Twist object used for pushing to nav node
from geometry_msgs.msg import Twist

# Detect bump events
from kobuki_msgs.msg import BumperEvent

# Detect laser events
from sensor_msgs.msg import LaserScan

# Detect odometry events
from nav_msgs.msg import Odometry

SLEEP_AMT = 0.1

kill = False
debug = True

vel_msg = Twist()

# Create publisher to turtlebot nav (queue_size determined arbitrarily)
nav_pub = rospy.Publisher('mobile_base/commands/velocity',
                          Twist,
                          queue_size=10)

# Inhibition flags
bump_inhibitor = False
command_inhibitor = False
keyword_inhibitor = False

# Default speed values
default_forward_velocity = 0.3  # assuming 0.3m/s
default_a_velocity = 1  # 1rad/s

# Keyboard variables
keys = None

# Odom variables
prev_pos = None

# Parser
parser = None

# Command Queue
commqueue = None

def commands_ready_cb():
    global command_inhibitor, keyword_inhibitor
    command_inhibitor = True
    keyword_inhibitor = False

def update_velocity_cb(twist_obj):
    global vel_msg
    if debug:
        print("Vel message received by command queue: {}".format(vel_msg))
    vel_msg = twist_obj

def command_done_cb(has_more_cmds):
    global command_inhibitor
    if debug:
        print("Command done!")
    # Continue executing commands if we have more
    command_inhibitor = has_more_cmds

def command_received_cb(cmd_obj):
    if debug:
        print("New command received: {}".format(cmd_obj))
    # Do nothing since the command is handled by command queue

def keyword_received_cb(kw):
    global keyword_inhibitor
    if kw is None:
        if debug:
            print("No keyword detected")
    else:
        if debug:
            print("Keyword received: {}".format(kw))
        print("YES MASTER?")
        keyword_inhibitor = True


def handle_that_bump(bump):
    """ Async bump handler

        Handles bumps by setting the bump inhibitor.

        Note:
            Is normally executed as a callback by the subscriber to the BumperEvent event.

    """
    global bump_inhibitor
    if debug:
        rospy.loginfo("bumped")
    if bump.state == BumperEvent.PRESSED:
        # Activate bump inhibition signal
        bump_inhibitor = True


def handle_that_odom(odom):
    """ Async odometry handler

        Handles odometry data by setting the pose for the command queue

        Note:
            Is normally executed as a callback by the subscriber to the Odometry event.

    """

    global prev_pos, command_inhibitor, commqueue

    if command_inhibitor:
        if debug:
            print(odom.pose.linear)
        # update pose\ only if we are executing a command
        pose = odom.pose.pose
        commqueue.update_pose(pose)


def set_vel(x=0.0, y=0.0, z=0.0, ax=0.0, ay=0.0, az=0.0):
    """Set velocity Twist object to be published for this time-step 

    Args:
        x: Linear velocity.
        y: Not used. 
        z: Not used.
        ax: Not used.
        ay: Not used.
        az: Angular velocity.

    Return:
        Twist object representing the velocities.

    Notes:
        Sets the global vel_msg velocities.

    """
    vel_msg.linear.x = x
    vel_msg.linear.y = y
    vel_msg.linear.z = z
    vel_msg.angular.x = ax
    vel_msg.angular.y = ay
    vel_msg.angular.z = az
    if False and debug:
        rospy.loginfo("SET VELOCITY")
        rospy.loginfo('x = %f, az = %f' % (x, az))
    # print str(vel_msg)
    return vel_msg


def do_bump():
    """ Process a bump inhibition state

        Stops the robot.
        If user presses "x" it will back up and continue.

        Note:
            Is normally executed when bump_inhibitor == True.

    """

    global bump_inhibitor
    rospy.loginfo('Bumped, press "x" to back up')
    set_vel()  # stop

    # Check if user presses 'x' to back up
    if keys.hasKey():
        if keys.key == 'x':
            linear_vel, angular_vel = keys.moveBindings[keys.key]
            set_vel(x=default_forward_velocity * linear_vel,
                    az=default_a_velocity * angular_vel)
            bump_inhibitor = False


def do_keys():
    """ Process a valid keypress event.

        Depending on the key, will cause the robot to move.
        Keys and actions defined in 'keyboard.py'.

        Note:
            Not called when bump_inhibitor state is true.

    """

    # Get keyboard input
    key = keys.key
    rospy.loginfo('Doing what the human says: ' + str(key))
    linear_vel, angular_vel = keys.moveBindings[key]
    # Move robot
    set_vel(x=default_forward_velocity * linear_vel,
            az=default_a_velocity * angular_vel)


def do_idle():
    # Move in circle by default
    set_vel(x=default_forward_velocity,
            az=default_a_velocity)


def do_command():
    # Keep calling process_commands until the done callback is called with no more commands in queue
    global commqueue
    commqueue.process_commands()


def do_keyword():
    # Stop the robot and wait for command
    set_vel()


# All bot logic goes here
def do_bot_logic():
    """ Performs a certain behavior based on set inhibition states.

    Priorities are determined by whichever inhibitor comes first in the
    if statements. Each action sets a velocity, and the function then
    publishes the desired velocity to the robot.

    """

    # Check for inhibition signals
    global bump_inhibitor, keyword_inhibitor, command_inhibitor
    if bump_inhibitor:
        # Do bumped logic
        do_bump()
    elif keys.hasKey():  # poll for keyboard input
        # Do key input
        do_keys()
    elif keyword_inhibitor:
        do_keyword()
    elif command_inhibitor:
        do_command()
    else:
        # No inhibition signals -> do idle operation
        do_idle()
    # Publish movement
    nav_pub.publish(vel_msg)


def start_bot():
    """ Maintains the "alive" status of the robot.

    Subscribes to all the required sensors and initializes all
    behavior handlers. Then runs the bot logic until we stop
    the program.

    """

    global kill, keys, commqueue, parser

    # Init keyboard
    keys = RoboKeyboardControl()
    # Init command queue
    commqueue = CommandQueue(default_forward_velocity, default_a_velocity,
                             commands_ready_cb, update_velocity_cb, command_done_cb, command_received_cb, keyword_received_cb)
    commqueue.start_listening()

    try:
        # Init subscribers
        rospy.Subscriber('mobile_base/events/bumper',
                        BumperEvent,
                        handle_that_bump)
        rospy.Subscriber('/odom', Odometry, handle_that_odom)
        # Init node
        rospy.init_node('cs5023_rip_opportunity', anonymous=True)

        # Keep program alive
        while not rospy.is_shutdown() and not kill:
            do_bot_logic()
            rospy.loginfo("-"*50)
            rospy.sleep(SLEEP_AMT)
    except rospy.ROSInterruptException:
        pass
    finally:
        commqueue.stop_listening()


if __name__ == "__main__":
    start_bot()
