from setuptools import find_packages
from setuptools import setup

setup(
    name='create_msgs',
    version='3.1.0',
    packages=find_packages(
        include=('create_msgs', 'create_msgs.*')),
)
