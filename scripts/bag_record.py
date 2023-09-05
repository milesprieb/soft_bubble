import subprocess, shlex, psutil

def main():

    print("Starting rosbag recording...")
    while True:
        command = "rosbag record /bubble_rgb1 /bubble_rgb2 /bubble_flow /Robotiq2FGripperRobotInput /tool0_pose"
        command = shlex.split(command)
        rosbag_proc = subprocess.Popen(command)

        if input() == 'q':
            for proc in psutil.process_iter():
                if "record" in proc.name() and set(command[2:]).issubset(proc.cmdline()):
                    proc.send_signal(subprocess.signal.SIGINT)
            rosbag_proc.send_signal(subprocess.signal.SIGINT)
            break

if __name__ == "__main__":
    main()