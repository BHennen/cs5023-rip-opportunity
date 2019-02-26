import rospy
import random as rand
import math

# Twist object used for pushing to nav node
from geometry_msgs.msg import Twist

# Detect bump events
from kobuki_msgs.msg import BumperEvent

# Detect laser events
from sensor_msgs.msg import LaserScan

# Create publisher to turtlebot nav (queue_size determined arbitrarily)
nav_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)

# Global twist object
vel_msg = Twist()

# Default sleep amount
SLEEP_AMT = 0.10

bumped = False

# Default speed values
default_forward_velocity = 0.3
default_a_velocity = 1.0

# Inhibition flags
bump_inhibitor = False
keyboard_inhibitor = False
obstacle_inhibitor = False
random_turn_inhibitor = False

# Bump variables
# bump_turn_steps = 0
bump_handler = None

# Laser variables
laser_min_distance = 0.50  # Distance that indicates object is immediately in front of robot
laser_danger_distance = 0.5
obstacle_handler = None

# Dev variables
kill = False
debug = True

#####################
# Utility Functions #
#####################


def to_radians(degrees):
	return float(degrees) * (2.0 * math.pi) / 360.0


################
# Bump Handler #
################


class BumpHandler:
	def __init__(self, direction):
		global debug
		# Bump left -> turn right
		if direction == 0:
			if debug:
				print "Turning right"
			turn_degrees = 90
			self.turn_direction = 1
		# Bump center -> turn around
		elif direction == 1:
			if debug:
				print "Turning around"
			turn_degrees = 180
			self.turn_direction = 1
		# Bump right -> turn left
		else:
			if debug:
				print "Turning left"
			turn_degrees = 90
			self.turn_direction = -1
		turn_rads = to_radians(turn_degrees)
		self.turn_steps = 1.0 + (turn_rads / default_a_velocity)
		if debug:
			print "radians: %f\nturn steps: %f" % (turn_rads, self.turn_steps)

	def do_turn(self):
		global bump_inhibitor
		if self.turn_steps > 0:
			if debug:
				print "Turning " + ("left" if self.turn_direction < 0 else "right") + (" for %d more turns" % self.turn_steps)
			set_vel(az=default_a_velocity * self.turn_direction)
			self.turn_steps -= 1
		else:
			if debug:
				print "Finished handling bump"
			bump_inhibitor = False


class ObstacleHandler:
	def __init__(self, min_ranges):
		self.ranges = min_ranges
		# If center danger or both left and right danger
		if self.ranges[1] < laser_danger_distance or self.ranges[0] < laser_danger_distance and self.ranges[2] < laser_danger_distance:
			# Turn around (TODO: +- 30 degrees)
			turn_rads = math.pi
		else:
			turn_rads = math.pi / 2
		# Otherwise turn away from closer obstacle
		self.turn_direction = -1 if self.ranges[2] < self.ranges[0] else 1
		self.turn_steps = default_a_velocity / turn_rads / SLEEP_AMT
		set_vel(az=self.turn_direction * default_a_velocity)

	def avoid(self):
		global debug
		global obstacle_inhibitor
		if self.turn_steps > 0:
			if debug:
				print "Avoiding for %d more time-steps" % self.turn_steps
			self.turn_steps -= 1
		else:
			obstacle_inhibitor = False

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
	if debug:
		print "SET VELOCITY"
		print 'x = %f, az = %f' % (x, az)
	# print str(vel_msg)
	return vel_msg


# Async bump handler - handle bumps
def handle_that_bump(bump):
	global bump_inhibitor, bump_handler
	# 1 = front, 2 = right, 0 = left
	# Note: no back bumper
	if bump.state == BumperEvent.PRESSED:
		# Activate bump inhibition signal
		bump_inhibitor = True
		direction = bump.bumper
		bump_handler = BumpHandler(direction)


# Async laser handler - handles lasers
def handle_that_laser(scan):
	global kill, obstacle_inhibitor, obstacle_handler
	if obstacle_inhibitor:
		return
	# Separate laser scan into thirds, determine if object is to left, right, or in front
	min_ranges = [scan.ranges[0], scan.ranges[320], scan.ranges[639]]
	if debug:
		print 'Ranges: %s' % str(min_ranges)

	obstacle_left = min_ranges[0] < laser_danger_distance
	obstacle_center = min_ranges[1] < laser_danger_distance
	obstacle_right = min_ranges[2] < laser_danger_distance

	if obstacle_left or obstacle_center or obstacle_right:
		obstacle_inhibitor = True
		obstacle_handler = ObstacleHandler(min_ranges)


def do_bump():
	bump_handler.do_turn()


def do_keys():
	# Get keyboard input
	# Move robot
	print 'Doing what the human says'


def do_obstacle_avoid():
	global obstacle_handler
	# Avoid obstacle
	print 'Avoiding the bad thing'
	obstacle_handler.avoid()


def do_random_turn():
	# Get random amount to turn by
	# Turn
	print 'Performing routine random turn'


def go_forward():
	if vel_msg.linear.x != default_forward_velocity:
		print 'Going forward'
		set_vel(x=default_forward_velocity)


# All bot logic goes here
def do_bot_logic():
	# Check for inhibition signals
	global bump_inhibitor, keyboard_inhibitor, obstacle_inhibitor, random_turn_inhibitor
	if bump_inhibitor:
		# Do bumped logic
		do_bump()
	elif keyboard_inhibitor:
		# Do key input
		do_keys()
	elif obstacle_inhibitor:
		# Avoid obstacle
		do_obstacle_avoid()
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
	global kill
	# Init subscribers
	rospy.Subscriber('mobile_base/events/bumper', BumperEvent, handle_that_bump)
	rospy.Subscriber('/scan', LaserScan, handle_that_laser)
	# Init node
	rospy.init_node('rip_opportunity_rover', anonymous=True)
	# Keep program alive
	while not rospy.is_shutdown() and not kill:
		print "Time=%s" % str(rospy.Time.now())
		print "----------------"
		do_bot_logic()
		print "----------------"
		rospy.sleep(SLEEP_AMT)


# Only run main function if this is primary thread
if __name__ == '__main__':
	try:
		start_bot()
	except rospy.ROSInterruptException: pass
