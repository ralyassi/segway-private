from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'rl_planner'



setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
            
        ('share/' + package_name, ['package.xml']),
    
    (os.path.join('share', package_name, 'launch'),
    glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='segway',
    maintainer_email='segway@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rl_planner_node = rl_planner.rl_planner_node:main'
        ],
    },
)
