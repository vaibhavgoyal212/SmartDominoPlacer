import subprocess
import time
import paramiko
import json
import os
import csv
from subprocess import Popen
from socket import gaierror

import navigation
# from PiMotorController import PiMotorController
from planning import Planner

# DICE Paths
CREDENTIAL_PATH = "ssh_credentials.json"
BASHRC_PATH = "$HOME/.bashrc"
DEVEL_SETUP_PATH = "$HOME/catkin_ws/devel/setup.bash"
MARKER_PUBLISHER_PATH = "$HOME/Desktop/sdp/SDP6/marker_publishing.py"
SPIN_PATH = "$HOME/Desktop/sdp/SDP6/spin.py"
# TurtleBot Paths
ROBOT_LAUNCH_PATH = "on_bot_launch.py"


class SDP6Launcher():
    def __init__(self,  credential_path=CREDENTIAL_PATH,
                 robot_launch_path=ROBOT_LAUNCH_PATH,
                 bashrc=BASHRC_PATH,
                 devel_setup=DEVEL_SETUP_PATH,
                 marker_publisher_path=MARKER_PUBLISHER_PATH,
                 spin_path=SPIN_PATH) -> None:
        # CONNECTION
        credentials = self._read_json(credential_path)
        if credentials is None:
            raise FileNotFoundError(
                f"Credential details not found: {credential_path}")
        self.ssh_client = self._set_up_ssh(
            credentials["hostname"],
            credentials["username"],
            credentials["password"]
        )
        credentials.clear()  # Computer Security be like
        # PATHS
        self.robot_launch_path = robot_launch_path
        self.bashrc_path = os.path.expandvars(bashrc)
        self.devel_setup_path = os.path.expandvars(devel_setup)
        self.marker_publisher_path = os.path.expandvars(marker_publisher_path)
        self.spin_path = os.path.expandvars(spin_path)
        # COMMANDS
        self.commands_table = self._read_json("./commands_config.json")
        self.commandfile_history = []
        self.carried_out_instructions = []
        # TODO: PROCESS POOL? 
        pass
        # MISC CHECKS
        self.expected_rostopics = list(csv.reader(
            open("expected_rostopics.csv", mode='r', encoding='utf-8')
        ))[0]
        # Motor Controllers
        # self.motor_controller = PiMotorController()
        # GOMAMON connection
        self.gomamon_ssh = paramiko.SSHClient()
        try:
            self.gomamon_ssh.connect(
                hostname="hostname", username="username", password="password"
            )
        except paramiko.SSHException:
            raise paramiko.SSHException("Gomamon Not Started.")
        # Planner
        self.planner = Planner()
        dominos = {'0': 100, '1': 100}  # let's pretend we have this many
        self.planner.load_dominos(dominos)
        # MAIN LOOP
        self._local_system_check()
        # TODO: remote system check? 
        print("Necessary files exist, going into main loop...")
        # laucnh
        self.robot_launch()
        self.running = True
        self._main_loop()  # waiting for commands

    def _read_json(self, filepath: str) -> dict:
        if not os.path.exists(filepath):
            print("File not found: " + filepath)
            return None
        with open(filepath, 'r', encoding='utf-8') as f:
            return json.load(f)

    def _set_up_ssh(self, hostname: str, username: str, password: str) -> paramiko.SSHClient:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy)
        # removed error handling to let exceptions be thrown automatically
        ssh.connect(
            hostname=hostname, username=username, password=password,
            timeout=5, auth_timeout=5, banner_timeout=5
        )
        _, stdout, _ = ssh.exec_command(
            "echo 'yes'"
        )
        response = stdout.read().decode('utf-8').strip()
        if response == 'yes':
            return ssh
        else:
            raise paramiko.SSHException(
                f"Unexpected return string from ssh client: {response}")

    def _remote_file_exists(self, filepath: str) -> bool:
        if self.ssh_client is not None:
            _, stdout, _ = self.ssh_client.exec_command(
                f"test -f {filepath}"
            )
            return stdout.channel.recv_exit_status() == 0
        else:
            print("Remote Connection not present.")
            return False

    def _clean_up_remote_dir(self, filepath: str="./commands/diceInbox/") -> bool:
        '''
        Cleans up the directory. Returns true if directory is empty after clean up. 
        '''
        histories = self._check_inbox(remote_filepath=filepath)
        for filepath in histories:
            self.ssh_client.exec_command(
                f"rm -f {filepath}"
            )
            print(f"Removed {filepath} in command history")
        return self._check_inbox(remote_filepath=filepath) == []

    def _check_roscore_status(self) -> bool:
        '''
        Can be used to report back the status of the system back to the user. 
        '''
        if self.ssh_client is None:
            print("SSH Connection Not Present. Roscore Status Unknown.")
            return False
        stdin, stdout, stderr = self._execute_cmd_with_source("rostopic list", True)
        for topic in stdout.readlines():
            if "/rosout" in topic:
                return True
        if "ERROR" in stderr.read().decode('utf-8').strip():
            return False
        print("Unable to explicitly determine roscore status, returning False...")
        return False

    def _local_system_check(self):
        files_tobe_checked = [
            self.bashrc_path, self.devel_setup_path, self.marker_publisher_path, self.spin_path
        ]
        missing_files = []
        for file in files_tobe_checked:
            if not os.path.exists(file):
                missing_files.append(file)
        if missing_files != []:
            raise FileExistsError(f"{missing_files} is missing.")

    def _execute_script_with_source(self, filepath: str, remote: bool, wait_for_finish=False, executor: str="python3") -> any:
        '''
        Execute specified scritp (default is python file), on remote shell or local.
        Basically a wrapper for self._execute_cmd_with_source().
        '''
        if remote:
            if self.ssh_client is None:
                raise paramiko.SSHException(
                    "Cannot execute scripts on robot with no SSH connection.")
            if not self._remote_file_exists(filepath):
                raise FileNotFoundError(
                    f"Remote File Not Found on TurtleBot: {filepath}")
        else:
            if not os.path.exists(filepath):
                raise FileNotFoundError(
                    f"Local File Not Found on DICE: {filepath}"
                )
        print(f"{executor} {filepath}")
        return self._execute_cmd_with_source(
            f"{executor} {filepath}", remote=remote, wait_for_finish=wait_for_finish
        )

    def _execute_cmd_with_source(self, command: str, remote: bool, wait_for_finish=False) -> any:
        '''
        local: Execute command line with source bashrc and source devel setup bash
        remote: Execute command line with source /opt/ros/kinetic/setup.bash

        Returns 3 channels (in out err) if remote. 
        Local: returns process if do not wait for finish, returns returncode of proc if wait_for_finish is set.
        '''
        if remote:
            if self.ssh_client is None:
                raise paramiko.SSHException("Cannot execute remote commands if ssh client is not set up.")
            return self.ssh_client.exec_command(
                "source /opt/ros/kinetic/setup.bash"
                " && source ./catkin_ws/devel/setup.bash"
                f" && {command}"
            )
        else:
            proc = Popen(
                f"source {self.bashrc_path}"
                f" && source {self.devel_setup_path}"
                f" && {command}",
                shell=True
            )
            if not wait_for_finish:
                return proc
            return proc.wait()

    def robot_launch(self) -> bool:
        '''
        called by user commands to bring up the robot. 
        Return true after seeing all expected topics from bringup. 
        '''
        print("Launching the robot...")
        stdin, stdout, stderr = self._execute_script_with_source(self.robot_launch_path, remote=True)
        launched = False
        while not launched:
            stdin, stdout, stderr = self._execute_cmd_with_source("rostopic list", True)
            launched_topics = [topic.strip() for topic in stdout.readlines()]
            print(f"Launched topics: {launched_topics}")
            all_topics = True
            for topic in self.expected_rostopics:
                if topic not in launched_topics:
                    all_topics = False
                    print(f"Unlaunched topic: {topic}")
            if all_topics:
                launched = True
        return True

    def slam_launch(self) -> bool:
        '''
        called by user commands to map the room. 
        Return true upon completed execution, not guaranteeing success.
        '''
        print("Launching autonomous SLAM to map the room...")
        if not self._check_roscore_status():
            print("SLAM Launch failed: Roscore is not running.")
            return False
        self._execute_cmd_with_source(
            "roslaunch ros_autonomous_slam autonomous_explorer.launch", remote=False
        )
        # proc = self._execute_script_with_source(self.marker_publisher_path, remote=False)
        return True

    def save_map(self, mapname: str) -> bool:
        '''
        called by user commands to save the map.
        Return true if map file (pgm and yaml) exist on pi. 
        
        Assumption: roscore has been running and the robot has mapped things.
        '''
        print(f"Saving the map as {mapname}...")
        expanded_path = f"./data/maps/{mapname}"
        full_map_path_yaml = expanded_path + ".yaml"
        full_map_path_pgm = expanded_path + ".pgm"
        if os.path.exists(full_map_path_yaml):
            raise FileExistsError(f"Map file with the same name present: {full_map_path_yaml}")
        return_code = self._execute_cmd_with_source(
            f"rosrun map_server map_saver -f {expanded_path}", remote=False, wait_for_finish=True
        )
        if not os.path.exists(full_map_path_yaml) or not os.path.exists(full_map_path_pgm):
            raise FileNotFoundError(f"Map file to be saved are not saved properly: {expanded_path}")
        # copy to pi
        sftp = self.ssh_client.open_sftp()
        sftp.put(full_map_path_yaml, full_map_path_yaml)
        sftp.put(full_map_path_pgm, full_map_path_pgm)
        sftp.close()
        return self._remote_file_exists(full_map_path_yaml) and self._remote_file_exists(full_map_path_pgm)

    def load_map(self, mapname: str) -> bool:
        '''
        Load a presaved map. Ideally followed by a localization command.
        Assumption: roscore has been running. 
        '''
        print(f"Loading map: {mapname}")
        expanded_path = f"./data/maps/{mapname}"
        full_map_path = expanded_path + ".yaml"
        if not os.path.exists(full_map_path):
            raise FileNotFoundError(f"Map file {mapname} not present: {full_map_path}")
        self._execute_cmd_with_source(
            f"roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:={full_map_path}", remote=False, wait_for_finish=False
        )
        time.sleep(3)
        self.localize()
        return True

    def localize(self) -> bool:
        """
        In a map (presumably loaded from saved maps), run /global_localiztion service and call spin.py
        
        Assumption: roscore

        Return: return code of spin.py is 0. 
        """
        print(f"Localizing...")
        global_localization = self._execute_cmd_with_source(
            f"rosservice call /global_localization", remote=False, wait_for_finish=True
        )
        if not global_localization:
            raise RuntimeError("Global Localization Failed.")
        navigation.spin()
        return True
        # return self._execute_script_with_source(  # TODO: spin.py going to be imported instead
        #     self.spin_path, remote=False, wait_for_finish=True, executor="python3"
        # ) == 0

    def navigate(self, pixel_x: str, pixel_y: str, rotation: str) -> bool:
        """
        Navigate to the specified location on map. Calls function in navigation.py

        Assumption: roscore

        Returns only after goal reached, True. 
        """
        transformed_args = navigation.parseAppInput(float(pixel_x), float(pixel_y), 0.0)
        # TODO: wait for publish_goal to remove while or call it as command line instead
        return navigation.publish_goal(transformed_args[0], transformed_args[1], float(rotation))

    def place_now(self, text: str) -> bool:
        '''
        The function to start placing rows after user has manually moved the robot or 
        the robot has navigated to desired position. 
        '''
        navigation.move_motor(0, 0, 0)  # stop any movements
        # convert planner code. 
        domino_matrix = self.planner.set_mission(text)
        truth_values = []
        for domino_row in domino_matrix:
            truth_row = [entry == '0' for entry in domino_row]
            truth_values.append(truth_row)
        # place, raise, move, stop, lower loop
        for truth_row in truth_values:
            # convert truth_row to 01001, call it as arg to gomamon
            zeroones = ["1" if truth else "0" for truth in truth_row]
            zeroonestring = "".join(zeroones)
            # self.motor_controller.place_row(truth_row)
            self.gomamon_ssh.exec_command(
                f"python3 ./sdp/PiMotorController.py {zeroonestring}"
            )
            # self.motor_controller.raise_wall()
            self.gomamon_ssh.exec_command(
                "python3 ./sdp/PiMotorController.py raise_wall"
            )
            navigation.move_motor(1, 0, 1)
            time.sleep(1)
            navigation.move_motor(0, 0, 0)
            # self.motor_controller.lower_wall()
            self.gomamon_ssh.exec_command(
                "python3 ./sdp/PiMotorController.py lower_wall"
            )
        # tell the app
        return self._write_void_file("plan_finished")

    def clean_up(self) -> bool:
        return self._clean_up_remote_dir()

    def execute_plan(self, text: str, pix_x: str, pix_y: str, rot: str) -> bool:
        '''
        The function to be reflected on to receive full user input. 

        user-input-text-to-be-printed,pixel-coordinate-x,pixel-coordinate-y,rotation

        returns true after notifying the app by writing the blank file. 
        '''
        self.navigate(pix_x, pix_y, rot)
        navigation.move_motor(0, 0, 0)  # stop
        # navigation.move_motor(0, 0, 0)  # stop
        # navigation.move_motor(1, 0, 1) # move one step
        return self.place_now(text)

    def _check_inbox(self, remote_filepath="./commands/diceInbox/") -> list:
        '''
        Reads the dice inbox folder on pi and return a list of files inside. 

        TODO: delete the file once received message? 阅后即焚...? get rid of GC

        e.g. [/paths/app_to_dice_2023-03-20-18-40-25.csv, ...]

        Assumes ssh client is up. 
        '''
        stdin, stdout, stderr = self.ssh_client.exec_command(
            f"ls {remote_filepath}"
        )
        return [remote_filepath + filename.strip() for filename in stdout.readlines()]
        # proc = subprocess.run(
        #     ["ls", "./mockPiHome/commands/diceInbox/"], capture_output=True
        # )
        # print(proc.stdout)

    def _write_void_file(self, name: str, writepath: str="./commands/appInbox/") -> bool:
        '''
        Writes a blank file to path. Information is in filename (with time stamp). 

        Args: name: name of file without suffix (.txt)

        Returns if the remote file is then existing. 
        '''
        timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
        name_with_timestamp = name + "_" + timestamp + ".txt"
        full_path = os.path.join(writepath + name_with_timestamp)
        stdin, stdout, stderr = self.ssh_client.exec_command(
            f"touch {full_path}"
        )
        return self._remote_file_exists(full_path)
    
    def _filter_new_commands(self, commands: list) -> list:
        new_commands = []
        for commandfile in commands:
            if not commandfile in self.commandfile_history:
                new_commands.append(commandfile)
        return new_commands

    def _commands_parser(self, command_file: str) -> dict:
        '''
        Opens the command file, parse the input and returns a dict of commands and their args. 

        Assumes self.ssh_client is up. 

        Put the executed command file into commands history
        '''
        sftp = self.ssh_client.open_sftp()
        commands = list(csv.reader(
            sftp.open(command_file, mode='r')
        ))
        sftp.close()
        # print(commands)
        instructions = {}
        for command in commands:
            if len(command) == 1:
                instructions[self.commands_table[command[0]]] = []
            elif len(command) > 1:
                instructions[self.commands_table[command[0]]] = command[1:]
            else:
                raise ValueError(f"Ill-formed command: {command}")
        self.commandfile_history.append(command_file)
        return instructions
    
    def _action(self, method_name: str, args: list=None, caller: str="mock command"):
        self.carried_out_instructions.append(
            f"{method_name} in {caller}: {args}"
        )
        getattr(self, method_name)(*args)

    def _main_loop(self):   
        '''
        The main loop waiting for commands. 
        Periodically checking filepath? tbd
        '''
        debug_timer = 500
        while self.running:
            command_files = self._check_inbox()
            # filter out new commands
            new_commands = self._filter_new_commands(command_files)
            print(new_commands)
            for command_file in new_commands:
                instructions = self._commands_parser(command_file)
                print(instructions)
                for (method_name, args) in instructions.items():
                    self._action(method_name, args, command_file)
            time.sleep(2)
            debug_timer -= 2
            if debug_timer < 0:
                self.running = False
        # END LOOP
        print(self.carried_out_instructions)
        # Clean up command history
        self.clean_up()


if __name__ == "__main__":
    launcher = SDP6Launcher()
    # launcher.robot_launch()
    # launcher._exec_with_source("ls -a")
    # print(launcher._check_roscore_status())
    # robot = launcher.robot_launch()
    # print(robot)
    # slam = launcher.slam_launch()
    # print(slam)
    # time.sleep(60)
    # savemap = launcher.save_map("auto_1")
    # print(savemap)
    # launcher._check_inbox()
    # SDP6Launcher._check_inbox()

