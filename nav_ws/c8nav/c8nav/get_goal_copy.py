# import yaml
# import os
# import subprocess
# import argparse
# import sys

# def main():
#     parser = argparse.ArgumentParser()
#     parser.add_argument('--name', required=True, help='Goal name to send (e.g., sofa)')
#     args = parser.parse_args()

#     yaml_path = "maps/saved_goals.yaml"
#     if not os.path.exists(yaml_path):
#         print(f'Error: Goal file not found at {yaml_path}')
#         sys.exit(1)

#     with open(yaml_path, 'r') as f:
#         goals = yaml.safe_load(f)

#     print(f)

#     if args.name not in goals:
#         print(f'Error: Goal name "{args.name}" not found in file.')
#         print(f'Available goals: {list(goals.keys())}')
#         sys.exit(1)

#     goal = goals[args.name]
#     x, y, theta = goal['x'], goal['y'], goal['theta']
#     print(f'Sending goal "{args.name}": x={x}, y={y}, theta={theta}')

#     try:
#         subprocess.run([
#             'ros2', 'run', 'c8nav', 'send_goal_qr_code',
#             '--x', str(x), '--y', str(y), '--theta', str(theta)
#         ], check=True)
#     except subprocess.CalledProcessError as e:
#         print(f'Failed to execute send_goal.py: {e}')
#         sys.exit(1)

# if __name__ == '__main__':
#     main()
