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
laser_danger_distance = 0.75
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
    def init(self, 
             linear_velocity = 0, # Default turn has no forward component
             angular_velocity = default_a_velocity):
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity

    # Record current time and set velocity to begin the turn
    def start_turn(self, turn_rads, linear_velocity=None, angular_velocity=None):
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
            print 'Rotating %f degs' % math.degrees(turn_rads)

    # Continue the current turn until we have reached the desired turn radians
    # Returns True when done turning, False if still turning
    def continue_turn(self):
        if self.current_angle < self.turn_rads: 
            # Set velocity to continue turn
            set_vel(x=self.linear_velocity,
                    az=self.angular_velocity)

            # Integrate angular velocity based on the time since we started turning to determine angle
            t1 = rospy.Time.now().to_sec()
            self.current_angle = default_a_velocity * (t1 - self.t0)

            if debug:
                print "Rotated %f deg" % math.degrees(self.current_angle) \
                                         * self.turn_direction
            return False
        else: 
            # Done turning
            self.stop_turn()  
            return True

    # Stop the robot from turning
    def stop_turn(self):
        set_vel() 


class ObstacleHandler:
    def __init__(self):
        global turn_handler

        self.turn_direction = None
        self.turn_handler = turn_handler

    # Initialize variables for turning
    def start(self, ranges):
        global escape_inhibitor

        if escape_inhibitor: 
            # Setup turn to face away from symmetric obstacle
            random_variance = math.radians(rand.randint(-30, 30))
            turn_rads = math.pi + random_variance
            self.turn_handler.start_turn(turn_rads)
        else: 
            # Set direction to turn away from asymmetric obstacle
            self.turn_direction = -1 if ranges[2] < ranges[0] else 1

    # Continue turn away from obstacle 180 +- 30 deg
    def escape(self):
        global escape_inhibitor

        if self.turn_handler.continue_turn():
            # Done turning
            escape_inhibitor = False
    
    # Turn away from the obstacle
    def avoid(self): 
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
        print 'Random turn from (%f to %f) deg will be: %f' % self.min_degs, self.max_degs, turn_degs
        turn_handler.start_turn(turn_rads)

        if debug:
            print 'Randomly rotating %f degs' % math.degrees(turn_rads)

    def turn(self):
        """Function that is called when wanting to continue turning."""
        
        global random_turn_inhibitor

        if self.turn_handler.continue_turn():
            # Done turning
            random_turn_inhibitor = False


#########################
# Basic Functionalities #
#########################


# Set velocity Twist object to be published for this time-step
def set_vel(x=0.0, y=0.0, z=0.0, ax=0.0, ay=0.0, az=0.0):
    vel_msg.linear.x = x
    vel_msg.linear.y = y
    vel_msg.linear.z = z
    vel_msg.angular.x = ax
    vel_msg.angular.y = ay
    vel_msg.angular.z = az
    if False and debug:
        print "SET VELOCITY"
        print 'x = %f, az = %f' % (x, az)
    # print str(vel_msg)
    return vel_msg


# Async bump handler - handle bumps
def handle_that_bump(bump):
    global bump_inhibitor
    if debug:
        print "bumped"
    if bump.state == BumperEvent.PRESSED:
        # Activate bump inhibition signal
        bump_inhibitor = True


# Async laser handler - handles lasers
def handle_that_laser(scan):
    global kill, escape_inhibitor, avoid_inhibitor, obstacle_handler
    if escape_inhibitor:
        return
    # Separate laser scan into thirds, determine if object is to left, right, or in front
    ranges = [min(scan.ranges[0:219]),
              min(scan.ranges[220:430]),
              min(scan.ranges[431:])]
    if debug:
        print_out = 'Ranges: \tleft = ' + str(ranges[0]) \
                    + "\n\t\tmiddle = " + str(ranges[1]) \
                    + "\n\t\tright = " + str(ranges[2])
        # print print_out

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
    global prev_pos, random_turn_inhibitor, random_turn_handler
    pose = odom.pose.pose
    curr_pos = pose.position
    if prev_pos is None:
        prev_pos = curr_pos
    else:
        dx = (curr_pos.x - prev_pos.x)
        dy = (curr_pos.y - prev_pos.y)
        dist = math.hypot(dx, dy)
        # If we have traveled a foot or more
        if dist >= 0.3048:
            prev_pos = curr_pos
            random_turn_inhibitor = True
            random_turn_handler.start()


def do_bump():
    global bump_inhibitor
    print 'Bumped, press "x" to back up'
    set_vel()  # stop
    if keys.hasKey():
        if keys.key == 'x':
            linear_vel, angular_vel = keys.moveBindings[keys.key]
            set_vel(x=default_forward_velocity * linear_vel,
                    az=default_a_velocity * angular_vel)
            bump_inhibitor = False


def do_keys():
    # Get keyboard input
    key = keys.key
    print 'Doing what the human says: ' + str(key)
    linear_vel, angular_vel = keys.moveBindings[key]
    # Move robot
    set_vel(x=default_forward_velocity * linear_vel,
            az=default_a_velocity * angular_vel)


def do_escape_obstacle():
    global obstacle_handler
    # escape obstacle
    print 'Escaping the bad thing'
    obstacle_handler.escape()


def do_avoid_obstacle():
    global obstacle_handler
    # Avoid obstacle
    print 'Avoiding the bad thing'
    obstacle_handler.avoid()


def do_random_turn():
    global random_turn_handler
    # Get random amount to turn by
    # Turn
    print 'Doing random turn'
    random_turn_handler.turn()


def go_forward():
    print 'Going forward'
    set_vel(x=default_forward_velocity)


# All bot logic goes here
def do_bot_logic():
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


# Maintains the "alive" status of the robot
def start_bot():
    global kill, keys, obstacle_handler, random_turn_handler, start_time
    # Init subscribers
    rospy.Subscriber('mobile_base/events/bumper',
                     BumperEvent,
                     handle_that_bump)
    rospy.Subscriber('/scan', LaserScan, handle_that_laser)
    rospy.Subscriber('/odom', Odometry, handle_that_odom)
    # Init node
    rospy.init_node('rip_opportunity_rover', anonymous=True)
    start_time = rospy.Time.now().to_sec()
    # Init keyboard
    keys = RoboKeyboardControl()
    turn_handler = TurnHandler()
    obstacle_handler = ObstacleHandler()
    random_turn_handler = RandomTurnHandler()

    # Keep program alive
    while not rospy.is_shutdown() and not kill:
        do_bot_logic()
        print "----------------"
        rospy.sleep(SLEEP_AMT)


# Only run main function if this is primary thread
if __name__ == '__main__':
    try:
        start_bot()
    except rospy.ROSInterruptException:
        pass
