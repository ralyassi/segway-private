from setuptools import setup, find_packages

package_name = 'cmd_vel_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='cmd_vel throttle/republisher',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'throttle_node = cmd_vel_tools.throttle_node:main',
        ],
    },
)
