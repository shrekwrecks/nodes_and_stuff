from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'ackermann_to_twist'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
                # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akdommers',
    maintainer_email='akdommers@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ackermann_to_twist = ackermann_to_twist.ackermann_to_twist:main',
            'heading_to_angle = ackermann_to_twist.heading_to_angle:main',
            'odom_tf_pub = ackermann_to_twist.odom_tf_pub:main',
        ],
    },

    # Other parameters ...

)
