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

# Create publisher to turtlebot nav (queue_size determined arbitrarily)
nav_pub = rospy.Publisher('mobile_base/commands/velocity',
                          Twist,
                          queue_size=10)

# Global twist object
vel_msg = Twist()

# Default sleep amount
SLEEP_AMT = 0.1

bumped = False

# Default speed values
default_forward_velocity = 0.3 # assuming 0.3m/s
default_a_velocity = 1 # 1rad/s

# Inhibition flags
bump_inhibitor = False
keyboard_inhibitor = False
escape_inhibitor = False
avoid_inhibitor = False
random_turn_inhibitor = False

# Keyboard variables
keys = None

# Laser variables
# Distance that indicates object is immediately in front of robot
laser_danger_distance = 1.75
obstacle_handler = None

# Turn variables
turn_handler = None
random_turn_handler = None
random_turn_min_degs = -15
random_turn_max_degs = 15

# Odom variables
prev_pos = None

# Dev variables
kill = False
debug = True


class TurnHandler:
    """Handles the turning of the robot for a specified amount """

    def __init__(self, linear_velocity=0, angular_velocity=default_a_velocity):
        """Creates a TurnHandler which has specified velocities.
        
        Args:
            linear_velocity: The desired linear velocity that will be applied while turning.
            angular_velocity: The desired angular velocity that will be applied while turning.

        """
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.t0 = None
        self.current_angle = None
        self.turn_direction = None
        self.turn_rads = None

    # Record current time and set velocity to begin the turn
    def start_turn(self, turn_rads, linear_velocity=None, angular_velocity=None):
        """Starts a turn which will proceed until the given angle has been reached.
        
        Args:
            turn_rads: The desired angle at which the robot will turn to.
            linear_velocity: The desired linear velocity that will be applied while turning.
            angular_velocity: The desired angular velocity that will be applied while turning.

        Notes:
            Call this function once when starting the turn.
            Specifying linear_velocity or angular_velocity will update the value for future calls
            using this object.

        """
        global debug

        # Update turn values
        if linear_velocity is not None:
            self.linear_velocity = linear_velocity
        if angular_velocity is not None:
            self.angular_velocity = angular_velocity

        # Set initial turn state variables
        self.t0 = rospy.Time.now().to_sec()
        self.current_angle = 0
        self.turn_direction = -1 if turn_rads < 0 else 1
        self.turn_rads = abs(turn_rads)

        if debug:
            rospy.loginfo('Rotating %f degs' % math.degrees(turn_rads))

    # Continue the current turn until we have reached the desired turn radians
    # Returns True when done turning, False if still turning
    def continue_turn(self):
        """Continues a turn which will proceed until the previously specified angle has been reached.

        Return:
            True: When robot reaches specified angle. Stops the robot when done turning.
            False: When robot needs to continue turning.

        Notes:
            Call this function until it returns true.

        """
        if self.current_angle < self.turn_rads: 
            # Set velocity to continue turn
            set_vel(x=self.linear_velocity,
                    az=self.angular_velocity*self.turn_direction)

            # Integrate angular velocity based on the time since we started turning to determine angle
            t1 = rospy.Time.now().to_sec()
            self.current_angle = default_a_velocity * (t1 - self.t0)

            if debug:
                rospy.loginfo("Rotated %f deg" % math.degrees(self.current_angle) \
                                         * self.turn_direction)
            return False
        else: 
            # Done turning
            # self.stop_turn()
            return True

    def stop_turn(self):
        """Stops the robot from turning"""
        set_vel() 


class ObstacleHandler:
    """Handles turning the robot depending on sensor range data it receives."""

    def __init__(self):
        """Creates a new ObstacleHandler which has avoid and escape behavior."""

        global turn_handler

        self.turn_direction = None
        self.turn_handler = turn_handler

    # Initialize variables for turning
    def start(self, ranges):
        """Starts the obstacle avoidance behavior.

        Notes:
            This function is called once whenever an obstacle is first detected that is too close.
            The behavior chosen is dependant on whether or not the global escape_inhibitor is set.
            If escape_inhibitor == True, then the robot will escape.
            Otherwise it will avoid.

        """
        global escape_inhibitor

        if escape_inhibitor: 
            # Setup turn to face away from symmetric obstacle
            random_variance = math.radians(rand.randint(-30, 30))
            turn_rads = math.pi + random_variance
            self.turn_handler.start_turn(turn_rads)
        else: 
            # Set direction to turn away from asymmetric obstacle
            self.turn_direction = -1 if ranges[2] < ranges[0] else 1

    def escape(self):
        """Turns away from an obstacle 180 +- 30 deg.
        
        Notes:
            This should be called until the robot has turned the desired amount.
            It then sets the global variable escape_inhibitor to False.
        """
        global escape_inhibitor

        if self.turn_handler.continue_turn():
            # Done turning
            escape_inhibitor = False
    
    # Turn away from the obstacle
    def avoid(self):
        """Turns away from an obstacle.
        
        Notes:
            This should be called until the robot has turned so that the obstacle is no longer close.
        """
        set_vel(az=self.turn_direction * default_a_velocity)


class RandomTurnHandler:
    """Handles turning the robot a random amount."""

    def __init__(self, min_degs=random_turn_min_degs, max_degs=random_turn_max_degs):
        """Creates a RandomTurnHandler that is initialized to turn between specified the degrees.

        Args:
            min_degs (optional): The minimum amount the robot should turn (can be negative).
            max_degs (optional): The maximum amount the robot should turn (can be negative).
        Note:
            min_degs <= max_degs

            min_degs and max_degs if not provided are set at the top of the file.
        
        """
        global turn_handler

        self.min_degs = min_degs
        self.max_degs = max_degs
        self.turn_handler = turn_handler
    
    def start(self, min_degs=None, max_degs=None):
        """Function that is called to initialize the start of a turn.

        Args:
            min_degs (optional): The minimum amount the robot should turn (can be negative).
            max_degs (optional): The maximum amount the robot should turn (can be negative).
        Note:
            min_degs <= max_degs

            min_degs and max_degs set the class values for future calls without parameters.
            min_degs and max_degs if not provided are set upon initialization of the class.
        
        """
        global debug

        if min_degs is not None:
            self.min_degs = min_degs
        if max_degs is not None:
            self.max_degs = max_degs
        
        turn_degs = rand.randint(self.min_degs, self.max_degs)
        turn_rads = math.radians(turn_degs)
        rospy.loginfo('Random turn from (%f to %f) deg will be: %f' % (self.min_degs, self.max_degs, turn_degs))
        turn_handler.start_turn(turn_rads)

        if debug:
            rospy.loginfo('Randomly rotating %f degs' % math.degrees(turn_rads))

    def turn(self):
        """Function that is called when wanting to continue turning."""
        
        global random_turn_inhibitor

        if self.turn_handler.continue_turn():
            # Done turning
            random_turn_inhibitor = False


#########################
# Basic Functionalities #
#########################



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


def handle_that_laser(scan):
    """ Async laser handler

        Handles laser data by checking for obstacles and setting the appropriate inhibitor:
        either escape or avoid. Also starts the ObstacleHandler.

        Note:
            Is normally executed as a callback by the subscriber to the LaserScan event.

    """

    global kill, escape_inhibitor, avoid_inhibitor, obstacle_handler
    if escape_inhibitor is None or obstacle_handler is None:
        return

    # Separate laser scan into thirds, determine if object is to left, right, or in front
    # print(len(scan.ranges))
    num_ranges = len(scan.ranges)
    region1 = int(num_ranges/3)
    region2 = region1*2
    ranges = [min(scan.ranges[0:region1]),
              min(scan.ranges[region1+1:region2]),
              min(scan.ranges[region2:])]
    if debug:
        print_out = 'Ranges: \tleft = ' + str(ranges[0]) \
                    + "\n\t\tmiddle = " + str(ranges[1]) \
                    + "\n\t\tright = " + str(ranges[2])
        # print print_out

    # Determine for each third if there is an obstacle within the danger zone.
    obstacle_left = ranges[0] < laser_danger_distance
    obstacle_center = ranges[1] < laser_danger_distance
    obstacle_right = ranges[2] < laser_danger_distance

    if obstacle_left and obstacle_right:
        # symmetric obstacle
        escape_inhibitor = True
        # reset avoid inhibitor in case symmetric obstacle detected while avoiding
        avoid_inhibitor = False
        obstacle_handler.start(ranges)
    elif obstacle_center or obstacle_left or obstacle_right:
        # asymmetric obstacle
        if not avoid_inhibitor:
            avoid_inhibitor = True
            obstacle_handler.start(ranges)
        obstacle_handler.ranges = ranges
    else:
        avoid_inhibitor = False


# Async odom handler - handles the odom
def handle_that_odom(odom):
    """ Async odometry handler

        Handles odometry data by calculating the travelled distance, setting
        the random_turn_inhibitor, and starting the random turn.

        Note:
            Is normally executed as a callback by the subscriber to the Odometry event.

    """

    global prev_pos, random_turn_inhibitor, random_turn_handler

    if random_turn_handler is None:
        return
    
    # update pose and position
    pose = odom.pose.pose
    curr_pos = pose.position
    if prev_pos is None:
        prev_pos = curr_pos
    else:
        # calculate traveled distance
        dx = (curr_pos.x - prev_pos.x)
        dy = (curr_pos.y - prev_pos.y)
        dist = math.hypot(dx, dy)
        # If we have traveled a foot or more
        if dist >= 0.3048:
            prev_pos = curr_pos
            random_turn_inhibitor = True
            random_turn_handler.start()


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
    
    #Check if user presses 'x' to back up
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


def do_escape_obstacle():
    """ Process a escape inhibition state.

        Rotates 180 +- 30 degs away from symmetric obstacle.

        Note:
            Is normally executed when escape_inhibitor == True,
            and none of the above states are true.

    """
    
    global obstacle_handler
    # escape obstacle
    rospy.loginfo('Escaping the bad thing')
    obstacle_handler.escape()


def do_avoid_obstacle():
    """ Process an avoid obstacle inhibition state.

    Turns away from the obstacle until the obstacle is no longer 
    considered too close to the robot.

    Note:
        Is normally executed when avoid_inhibitor == True,
        and none of the above states are true.

    """

    global obstacle_handler
    # Avoid obstacle
    rospy.loginfo('Avoiding the bad thing')
    obstacle_handler.avoid()


def do_random_turn():
    """ Process an random turn inhibition state.

    Rotates randomly +- 15 degs. Called automatically after
    the robot travels forward ~1 foot.

    Note:
        Is normally executed when random_turn_inhibitor == True,
        and none of the above states are true.

    """
    global random_turn_handler
    # Get random amount to turn by
    # Turn
    rospy.loginfo('Doing random turn')
    random_turn_handler.turn()


def go_forward():
    """ The robot drives forward.

    Note:
        Is normally executed when none of the above states are true.

    """

    rospy.loginfo('Going forward')
    set_vel(x=default_forward_velocity)


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
    elif escape_inhibitor:
        # escape obstacle
        do_escape_obstacle()
    elif avoid_inhibitor:
        # avoid obstacle
        do_avoid_obstacle()
    elif random_turn_inhibitor:
        # Do random turn
        do_random_turn()
    else:
        # No inhibition signals -> go forward
        go_forward()
    # Publish movement
    nav_pub.publish(vel_msg)

def start_bot():
    """ Maintains the "alive" status of the robot.

    Subscribes to all the required sensors and initializes all
    behavior handlers. Then runs the bot logic until we stop
    the program.

    """

    global kill, keys, obstacle_handler, random_turn_handler, turn_handler
    # Init subscribers
    rospy.Subscriber('mobile_base/events/bumper',
                     BumperEvent,
                     handle_that_bump)
    rospy.Subscriber('/scan', LaserScan, handle_that_laser)
    rospy.Subscriber('/odom', Odometry, handle_that_odom)
    # Init node
    rospy.init_node('cs5023_rip_opportunity', anonymous=True)
    # Init keyboard
    keys = RoboKeyboardControl()
    turn_handler = TurnHandler()
    obstacle_handler = ObstacleHandler()
    random_turn_handler = RandomTurnHandler()

    # Keep program alive
    while not rospy.is_shutdown() and not kill:
        do_bot_logic()
        rospy.loginfo("----------------")
        rospy.sleep(SLEEP_AMT)


# Only run main function if this is primary thread
if __name__ == '__main__':
    try:
        start_bot()
    except rospy.ROSInterruptException:
        pass
