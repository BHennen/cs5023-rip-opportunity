
import math

# Ros packages
# Twist object used for pushing to nav node
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


class CommandHandler:
    def __init__(self):
        self.command = None
        self.prev_pose = None
        self.current_pose = None
        self.current_magnitude = 0
        self.goal_magnitude = None
        self.prev_yaw = None
        self.current_yaw = None

    def start_command(self, command, linear_velocity, angular_velocity, set_velocity_cb):
        """ Parses a given command and then calls the set_velocity_cb with a Twist object.

        command: Command - command which is parsed
        linear_velocity: float - linear speed to perform the command
        angular_velocity: float - angular speed to perform the command
        set_velocity_cb: function - Callback function which is passed a Twist object to perform the command.
        """
        print("Starting execution of command "+str(command))
        # Reset pose and accumulators
        self.command = command
        self.prev_pose = None
        self.current_pose = None
        self.current_magnitude = 0
        self.goal_magnitude = command.magnitude
        self.prev_yaw = None
        self.current_yaw = None

        # Set translational
        trans_vel_mult = 1 if command.f else -1 if command.b else 0
        # Set rotational
        ang_vel_mult = 1 if command.r else -1 if command.l else 0

        # If we want to go at an angle (using both translational and angular velocity)
        # and the goalpoint is set, calculate angular velocity to reach the goalpoint
        if trans_vel_mult and ang_vel_mult and command.magnitude:
            angular_velocity = self.__get_a_vel(
                command, linear_velocity, angular_velocity)

        # Send velocity to callback
        set_velocity_cb(Twist(x=trans_vel_mult * linear_velocity,
                              az=ang_vel_mult * angular_velocity))

    def continue_command(self, done_cb):
        """ Checks distance from origin position and calls done_cb when goal reached (if goal set)

        Does nothing if no goal set.

        done_cb: function - Callback function which is called when the command is done. No parameters
        """
        if self.goal_magnitude:
            self.__update_magnitude()
            if self.current_magnitude >= self.goal_magnitude:
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

    def __get_a_vel(self, command, linear_velocity, angular_velocity):
        """ Non-cardinal (arc) movement with desired endpoint
            Based on traveling left or right 45 degrees arc
            Future goal could be to implement command that would travel a distance at any angle
        """
        return math.sqrt(2) * linear_velocity / command.magnitude

    def __update_magnitude(self):
        if self.current_pose is None or self.prev_pose is None:
            # Return if odom hasn't updated our pose yet
            return
        if self.command.f or self.command.b:
            # calculate translational distance
            dx = (self.current_pose.position.x - self.prev_pose.position.x)
            dy = (self.current_pose.position.y - self.prev_pose.position.y)
            self.current_magnitude += math.hypot(dx, dy)
        else:
            # calculate rotational distance
            self.prev_yaw = self.current_yaw
            quat = self.current_pose.orientation
            (roll, pitch, yaw) = euler_from_quaternion(
                [quat.x, quat.y, quat.z, quat.w])
            self.current_yaw = yaw
            if self.prev_yaw is None:
                # first iteration with no previous, no need to calculate
                return

            # Accumulate angle traveled
            diff = abs(self.current_yaw - self.prev_yaw)
            angle = 360-diff if diff > 180 else diff
            self.current_magnitude += angle
