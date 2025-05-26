import yaml
import os
import subprocess
import argparse
import sys
import time

from c8nav.qr_code_aruco_create import docking
from c8nav.pose_estimate import publish_initial_pose, move_back

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', required=True, help='Goal name to send (e.g., sofa)')
    args = parser.parse_args()

    yaml_path = "maps/saved_goals.yaml"
    if not os.path.exists(yaml_path):
        print(f'Error: Goal file not found at {yaml_path}')
        sys.exit(1)

    with open(yaml_path, 'r') as f:
        goals = yaml.safe_load(f)

    print(f)

    if args.name not in goals:
        print(f'Error: Goal name "{args.name}" not found in file.')
        print(f'Available goals: {list(goals.keys())}')
        sys.exit(1)

    goal = goals[args.name]
    x, y, theta = goal['x'], goal['y'], goal['theta']
    print(f'Sending goal "{args.name}": x={x}, y={y}, theta={theta}')

    pick_up_goal = goals['pick_up']
    pick_up_x, pick_up_y, pick_up_theta = pick_up_goal['x'], pick_up_goal['y'], pick_up_goal['theta']
    try:
        move_back(1)
        publish_initial_pose(pick_up_x, pick_up_y, pick_up_theta)

        subprocess.run([
            'ros2', 'run', 'c8nav', 'send_goal',
            '--x', str(x), '--y', str(y), '--theta', str(theta)
        ], check=True)
        #  ## OPEN LID
        # print('Sleeping for 10 seconds to allow for pickup.')
        # time.sleep(5)
        # ## CLOSE LID
        publish_initial_pose(x, y, theta)

        print('Go back to pick up goal')
        subprocess.run([
            'ros2', 'run', 'c8nav', 'send_goal',
            '--x', str(pick_up_x), '--y', str(pick_up_y), '--theta', str(pick_up_theta), ], check=True)
        docking()
         ## OPEN LID
         ## CLOSE LID

       

    except subprocess.CalledProcessError as e:
        print(f'Failed to execute send_goal.py: {e}')
        sys.exit(1)

if __name__ == '__main__':
    main()
