#!/usr/bin/env python
import rospy
import random as rand
import math
from keyboard import RoboKeyboardControl

# Twist object used for pushing to nav node
from geometry_msgs.msg import Twist

# Detect bump events
from kobuki_msgs.msg import BumperEvent

# Detect laser events
from sensor_msgs.msg import LaserScan

# Detect odometry events
from nav_msgs.msg import Odometry

from speech_commands.command_parser import CommandParser
import math
import rospy
from keyboard import RoboKeyboardControl
from tf.transformations import euler_from_quaternion

# Twist object used for pushing to nav node
from geometry_msgs.msg import Twist

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

# Turn variables
turn_handler = None
commhandler = None


class CommandHandler:
    def __init__(self):
        self.command = None
        self.vel_msg = None
        self.current_distance = 0
        self.current_pose = None
        self.prev_pose = None
        self.goal_distance = None
        self.prev_yaw = None
        self.current_yaw = None
        self.total_angle = 0

    def start_command(self, command, linear_velocity=default_forward_velocity):
        self.command = command
        print("Starting execution of command "+str(command))

        # Set translational
        trans_vel = 1 if command.f else -1 if command.b else 0
        # Set rotational
        ang_vel_mult = 1 if command.r else -1 if command.l else 0
        ang_vel = self.__get_a_vel(command, linear_velocity)

        self.vel_msg = Twist(x=trans_vel * linear_velocity,
                             az=ang_vel_mult * ang_vel)
        self.goal_distance = command.magnitude

        # Reset pose and accumulators
        self.current_distance = 0
        self.current_pose = None
        self.prev_pose = None
        self.total_angle = 0
        self.prev_yaw = None
        self.current_yaw = None

    def continue_command(self):
        """ Checks distance from origin position

        """
        global command_inhibitor
        if command_inhibitor:
            # Update velocity
            set_vel(self.vel_msg)
            # Check if goal reached
            if self.goal_distance:
                self.__update_distance()
                if self.current_distance >= self.goal_distance:
                    command_inhibitor = False

    def update_pose(self, pose):
        """
        Called by async odom handler
        """
        if self.prev_pose is None:
            self.prev_pose = pose
        else:
            self.prev_pose = self.current_pose
        self.current_pose = pose

    def __get_a_vel(self, command, linear_velocity):
        """ Math

        """
        if command.magnitude != 0:
            # Non-cardinal (arc) movement
            return math.sqrt(2) * linear_velocity / command.magnitude
        else:
            # Stationary rotation
            return default_a_velocity

    def __update_distance(self):
        if self.current_pose is None or self.prev_pose is None:
            # Return if odom hasn't updated our pose yet
            return
        if self.command.f or self.command.b:
            # calculate translational distance
            dx = (self.current_pose.position.x - self.prev_pose.position.x)
            dy = (self.current_pose.position.y - self.prev_pose.position.y)
            self.current_distance += math.hypot(dx, dy)
        else:
            # calculate rotational distance
            self.prev_yaw = self.current_yaw
            quat = self.current_pose.orientation
            (roll, pitch, yaw) = euler_from_quaternion(
                [quat.x, quat.y, quat.z, quat.w])
            self.current_yaw = yaw
            if self.prev_yaw is None:
                return

            # Accumulate angle travelled
            diff = abs(self.current_yaw - self.prev_yaw)
            angle = 360-diff if diff > 180 else diff
            self.total_angle += angle


def handle_that_command(cmd):
    global command_inhibitor, commhandler, keyword_inhibitor

    if cmd is None:
        if debug:
            print("No command detected")
    else:
        if debug:
            print("Command received: {}".format(cmd))

        print("Doing command: "+str(cmd))
        # Unconditionally set inhibitor
        keyword_inhibitor = False
        commhandler.start_command(cmd)
        command_inhibitor = True


def handle_that_keyword(kw):
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

        Handles odometry data by setting the pose for the command handler

        Note:
            Is normally executed as a callback by the subscriber to the Odometry event.

    """

    global prev_pos, command_inhibitor, commhandler

    if command_inhibitor:
        # update pose\ only if we are executing a command
        pose = odom.pose.pose
        commhandler.update_pose(pose)


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
    commhandler.continue_command()


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
    # global bump_inhibitor, keyboard_inhibitor, escape_inhibitor, random_turn_inhibitor
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

    global kill, keys, turn_handler, commhandler
    # Init subscribers
    rospy.Subscriber('mobile_base/events/bumper',
                     BumperEvent,
                     handle_that_bump)
    rospy.Subscriber('/odom', Odometry, handle_that_odom)
    # Init node
    rospy.init_node('cs5023_rip_opportunity', anonymous=True)
    # Init keyboard
    keys = RoboKeyboardControl()
    commhandler = CommandHandler()

    # Keep program alive
    while not rospy.is_shutdown() and not kill:
        do_bot_logic()
        rospy.loginfo("-"*50)
        rospy.sleep(SLEEP_AMT)


if __name__ == "__main__":
    parser = CommandParser(command_callback=handle_that_command,
                           keyword_callback=handle_that_keyword, loop_until_command=True)
    parser.start()
    try:
        start_bot()
    except rospy.ROSInterruptException:
        pass
    parser.stop()
