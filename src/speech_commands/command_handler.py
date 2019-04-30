
import math
from collections import deque

from command_parser import CommandParser

# Ros packages
# Twist object used for pushing to nav node
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

_debug = True


class CommandQueue:
    def __init__(self, linear_velocity, angular_velocity,
                 commands_ready_cb, update_velocity_cb, command_done_cb, command_received_cb, keyword_received_cb=None,
                 grammar=None, keywords=None):
        self.commhandler = CommandHandler(linear_velocity, angular_velocity)
        self.parser = CommandParser(
            self.__cmd_received_cb,
            self.__kwd_received_cb,
            grammar,
            keywords)
        self.ext_cmd_cb = command_received_cb
        self.ext_kwd_cb = keyword_received_cb
        self.ext_vel_cb = update_velocity_cb
        self.ext_done_cb = command_done_cb
        self.ext_ready_cb = commands_ready_cb
        self.cmd_queue = deque()
        # Bools representing our state.
        self.running_cmd = False  # Running a single command. True while a command is running
        self.running_cmds = False  # Running multiple commands. True while started queue

    def start_listening(self):
        """ Start listening for keywords and commands """
        self.parser.start()

    def stop_listening(self):
        """ Stop listening for keywords and commands """
        self.parser.stop()

    def update_pose(self, pose):
        """ Updates pose of command handler; should be called by odom update """
        self.commhandler.update_pose(pose)

    def process_commands(self):
        """ Process current command queue.
        """
        if not self.running_cmd:
            self.__start_next_cmd()
        else:
            self.__continue_cmd()

    def change_velocities(self, linear_velocity, angular_velocity):
        self.commhandler.change_velocities(linear_velocity, angular_velocity)

    def __cmd_received_cb(self, command):
        """ Called by self.parser whenever a command is received 

        TODO: interactive feedback loop
        TODO: commands with no termination magnitude (stop, go forward, turn right, etc.)
              will never end during execution of queue
        """
        wait_for_more_cmds = False
        # Check if we're currently in middle of command(s)
        if self.running_cmds:
            # We're running commands and got interrupted...
            if not command or 'begin' in command.command_str:
                # Received invalid command, ignore it for now and continue with our current queue
                wait_for_more_cmds = False
            else:
                # Received valid command, clear current queue
                self.cmd_queue.clear()
                # Notify we are done with our commands
                self.__done_cb()
                # Initialize queue with current command
                self.__add_cmd_to_queue(command)
                wait_for_more_cmds = True
        else:
            # Not running any commands
            if not command:
                # Wait for valid command...
                wait_for_more_cmds = True
            else:
                # Received valid command
                if 'begin' in command.command_str:
                    # Received command to tell us to start with current queue
                    if self.__has_cmds():
                        # We have commands, so signal that we're ready to start running our commands
                        wait_for_more_cmds = False
                        self.__notify_commands_ready()
                    else:
                        # No commands were added yet, wait for more
                        wait_for_more_cmds = True
                else:
                    # Received normal command, add it to the queue
                    self.__add_cmd_to_queue(command)
                    wait_for_more_cmds = True

        # Signal the callback for the received command and see it they want to continue for more cmds
        ext_wait_for_more_cmds = self.ext_cmd_cb(command)
        # If they return None then we use our method of waiting for more commands
        wait_for_more_cmds = ext_wait_for_more_cmds if ext_wait_for_more_cmds is not None else wait_for_more_cmds
        if _debug:
            print(
                "CommandQueue- Waiting for more commands: {}".format(wait_for_more_cmds))
        return wait_for_more_cmds

    def __kwd_received_cb(self, keyword):
        """ Called by self.parser whenever keyword is received 
        """
        self.ext_kwd_cb(keyword)

    def __done_cb(self):
        """ Called whenever CommandHandler is done executing a command

        ext_done_cb is passed a bool indicating if there are more commands left in queue.
            -> True if more commands, False if no more commands
        """
        # Current command is done, check if more commands in queue
        self.running_cmd = False
        self.running_cmds = self.__has_cmds()
        self.ext_done_cb(self.running_cmds)

    def __set_vel_cb(self, twist_obj):
        """ Called continuously while CommandHandler is running a command 

        ext_vel_cb is called with Twist object with desired velocities of the command.
        """
        self.ext_vel_cb(twist_obj)

    def __notify_commands_ready(self):
        """ Called internally when command list has changed and is
            ready to be executed by whoever needs it. 
        """
        self.ext_ready_cb()

    def __add_cmd_to_queue(self, command):
        if _debug:
            print("CommandQueue- Added to queue: {}".format(command))
        self.cmd_queue.append(command)

    def __continue_cmd(self):
        if _debug:
            print("CommandQueue- Continuing command: {}".format(self.commhandler.command))
        self.commhandler.continue_command(self.__done_cb, self.__set_vel_cb)

    def __start_next_cmd(self):
        # Gets the next command and starts execution of it
        next_cmd = self.__next_cmd()
        if next_cmd:
            self.running_cmd = True
            self.running_cmds = True
            if _debug:
                print("CommandQueue- Starting command: {}".format(next_cmd))
            self.commhandler.start_command(next_cmd)
        else:
            # TODO what should happen if trying to start command with none left in queue
            raise Exception("trying to start command with none left in queue")

    def __next_cmd(self):
        # Remove and return first element from queue (FIFO)
        try:
            return self.cmd_queue.popleft()
        except IndexError:
            # Out of commands
            return None

    def __has_cmds(self):
        # Check if elements in queue
        return True if self.cmd_queue else False


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
        self.vel_msg = Twist()

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
        ang_vel_mult = -1 if self.command.r else 1 if self.command.l else 0

        # If we want to go at an angle (using both translational and angular velocity)
        # and the goalpoint is set, calculate angular velocity to reach the goalpoint
        calc_ang_vel = self.angular_velocity
        if trans_vel_mult and ang_vel_mult and self.command.magnitude:
            calc_ang_vel = self.__get_a_vel(
                self.command.magnitude, self.linear_velocity, self.angular_velocity)

        # Set velocity message
        self.vel_msg.linear.x = trans_vel_mult * self.linear_velocity
        self.vel_msg.angular.z = ang_vel_mult * calc_ang_vel

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
