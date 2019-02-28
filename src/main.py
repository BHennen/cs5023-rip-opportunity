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
nav_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)

# Global twist object
vel_msg = Twist()

# Default sleep amount
SLEEP_AMT = 0.1

bumped = False

# Default speed values
default_forward_velocity = 0.3
default_a_velocity = 1

# Inhibition flags
bump_inhibitor = False
keyboard_inhibitor = False
escape_inhibitor = False
avoid_inhibitor = False
random_turn_inhibitor = False

# Keyboard variables
keys = None

# Laser variables
laser_min_distance = 0.50  # Distance that indicates object is immediately in front of robot
laser_danger_distance = 0.75
obstacle_handler = None

# Turn variables
random_turn_handler = None

# Odom variables
prev_pos = None

# Dev variables
kill = False
debug = True


################
# Bump Handler #
################


class ObstacleHandler:
    def __init__(self):
        self.ranges = None
        self.turn_direction = None
        self.turn_rads = None
        self.t0 = None
        self.current_angle = None

    def start(self, ranges):
        self.ranges = ranges
        global escape_inhibitor

        if escape_inhibitor:
            random_variance = math.radians(rand.randint(-30, 30))
            self.turn_rads = math.pi + random_variance
            self.t0 = rospy.Time.now().to_sec()
            self.current_angle = 0

        self.turn_direction = -1 if self.ranges[2] < self.ranges[0] else 1

    def escape(self):
        global debug
        global escape_inhibitor

        if self.current_angle < self.turn_rads:
            set_vel(az=self.turn_direction * default_a_velocity)
            t1 = rospy.Time.now().to_sec()
            self.current_angle = default_a_velocity * (t1 - self.t0)
            if debug:
                print "Rotated %f deg" % math.degrees(self.current_angle)*self.turn_direction
        else:
            escape_inhibitor = False
            set_vel()  # reset vel

    def avoid(self):
        global debug
        global avoid_inhibitor

        set_vel(az=self.turn_direction * default_a_velocity)


class RandomTurnHandler:
    def __init__(self):
        self.turn_rads = None
        self.turn_direction = None
        self.current_angle = None
        self.t0 = None

    def start(self, turn_rads):
        global random_turn_inhibitor
        self.t0 = rospy.Time.now().to_sec()
        self.current_angle = 0
        self.turn_direction = -1 if turn_rads < 0 else 1
        self.turn_rads = abs(turn_rads)
        print 'Randomly rotating %f degs' % math.degrees(turn_rads)

    def turn(self):
        global debug
        global random_turn_inhibitor

        if self.current_angle < self.turn_rads:
            set_vel(az=self.turn_direction * default_a_velocity)
            t1 = rospy.Time.now().to_sec()
            self.current_angle = default_a_velocity * (t1 - self.t0)
            if debug:
                print "Rotated %f rads" % math.degrees(self.current_angle)*self.turn_direction
        else:
            random_turn_inhibitor = False
            set_vel()  # reset vel


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
    ranges = [min(scan.ranges[0:219]), min(scan.ranges[220:430]), min(scan.ranges[431:])]
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
        avoid_inhibitor = False  # reset avoid inhibitor in case symmetric obstacle detected while avoiding
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
            turn_degs = rand.randint(-15, 15)
            turn_rads = math.radians(turn_degs)
            print 'Random turn will be: %f' % turn_degs
            random_turn_inhibitor = True
            random_turn_handler.start(turn_rads)


def do_bump():
    global bump_inhibitor
    print 'Bumped, press "x" to back up'
    set_vel()  # stop
    if keys.hasKey():
        if keys.key == 'x':
            linear_vel, angular_vel = keys.moveBindings[keys.key]
            set_vel(x=default_forward_velocity * linear_vel, az=default_a_velocity * angular_vel)
            bump_inhibitor = False


def do_keys():
    # Get keyboard input
    key = keys.key
    print 'Doing what the human says: ' + str(key)
    linear_vel, angular_vel = keys.moveBindings[key]
    # Move robot
    set_vel(x=default_forward_velocity * linear_vel, az=default_a_velocity * angular_vel)


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


# time_now = rospy.Time.now().to_sec()
# if (time_now - start_time > 1):
# 	exit()


# Maintains the "alive" status of the robot
def start_bot():
    global kill, keys, obstacle_handler, random_turn_handler, start_time
    # Init subscribers
    rospy.Subscriber('mobile_base/events/bumper', BumperEvent, handle_that_bump)
    rospy.Subscriber('/scan', LaserScan, handle_that_laser)
    rospy.Subscriber('/odom', Odometry, handle_that_odom)
    # Init node
    rospy.init_node('rip_opportunity_rover', anonymous=True)
    start_time = rospy.Time.now().to_sec()
    # Init keyboard
    keys = RoboKeyboardControl()
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
