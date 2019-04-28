
import math

# Ros packages
# Twist object used for pushing to nav node
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class CommandHandler:
    def __init__(self, linear_velocity, angular_velocity):
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.command = None
        self.vel_msg = None
        self.current_distance = 0
        self.current_pose = None
        self.prev_pose = None
        self.goal_distance = None
        self.prev_yaw = None
        self.current_yaw = None
        self.total_angle = 0

    def start_command(self, command, linear_velocity=None):
        if linear_velocity is None:
            linear_velocity = self.linear_velocity
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

    def continue_command(self, set_velocity, done_cb):
        """ Checks distance from origin position

        set_velocity: function - Callback function which is passed a Twist object. Called continuously as command is performed.
        done_cb: function - Callback function which is called when the command is done. No parameters
        """
        # Update velocity
        set_velocity(self.vel_msg)
        # Check if goal reached
        if self.goal_distance:
            self.__update_distance()
            if self.current_distance >= self.goal_distance:
                done_cb()

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
            return self.angular_velocity

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
