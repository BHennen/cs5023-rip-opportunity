
import math

# Ros packages
# Twist object used for pushing to nav node
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


class CommandHandler:
    def __init__(self, linear_velocity, angular_velocity):
        self.command = None
        self.prev_pose = None
        self.current_pose = None
        self.current_magnitude = 0
        self.goal_magnitude = None
        self.prev_yaw = None
        self.current_yaw = None
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity

    def start_command(self, command):
        """ Parses a given command and then calls the set_velocity_cb with a Twist object.

        command: Command - command which is parsed
        linear_velocity: float - linear speed to perform the command
        angular_velocity: float - angular speed to perform the command        
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
        self.__update_vel_msg()

    def continue_command(self, done_cb, set_velocity_cb):
        """ Checks distance from origin position and calls done_cb when goal reached (if goal set)
        set_velocity_cb is called with Twist object with desired velocities of the command. 

        done_cb: function - Callback function which is called when the command is done. No parameters
        set_velocity_cb: function - Callback function which is passed a Twist object to perform the command.
        """
        if self.goal_magnitude:
            self.__update_magnitude()
            if self.current_magnitude >= self.goal_magnitude:
                # Goal reached, send done signal and return to not set any velocity
                done_cb()
                return

        set_velocity_cb(self.vel_msg)

    def change_velocities(self, linear_velocity, angular_velocity):
        """ Change the velocities that the commands (including currently executing) are processed with. """
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.__update_vel_msg()

    def update_pose(self, pose):
        """
        Called by async odom handler
        """
        if self.prev_pose is None:
            self.prev_pose = pose
        else:
            self.prev_pose = self.current_pose
        self.current_pose = pose

    def __update_vel_msg(self):
        # Set translational
        trans_vel_mult = 1 if self.command.f else -1 if self.command.b else 0
        # Set rotational
        ang_vel_mult = 1 if self.command.r else -1 if self.command.l else 0

        # If we want to go at an angle (using both translational and angular velocity)
        # and the goalpoint is set, calculate angular velocity to reach the goalpoint
        calc_ang_vel = self.angular_velocity
        if trans_vel_mult and ang_vel_mult and self.command.magnitude:
            calc_ang_vel = self.__get_a_vel(
                self.command.magnitude, self.linear_velocity, self.angular_velocity)

        # Set velocity message
        self.vel_msg = Twist(x=trans_vel_mult * self.linear_velocity,
                             az=ang_vel_mult * calc_ang_vel)

    def __get_a_vel(self, magnitude, linear_velocity, angular_velocity):
        """ Non-cardinal (arc) movement with desired endpoint
            Based on traveling left or right 45 degrees arc
            Future goal could be to implement command that would travel a distance at any angle
        """
        return math.sqrt(2) * linear_velocity / magnitude

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
            angle = (2*math.pi)-diff if diff > math.pi else diff
            self.current_magnitude += angle
