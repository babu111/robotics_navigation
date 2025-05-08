from setuptools import find_packages, setup

package_name = 'c8nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gixadmin',
    maintainer_email='zc096373@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_goal = c8nav.send_goal:main',
            'save_current_pose = c8nav.save_current_pose:main',
            'get_goal = c8nav.get_goal:main'
        ],
    },
)
