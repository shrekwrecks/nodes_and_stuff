from setuptools import find_packages, setup

package_name = 'ackermann_to_twist'

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
    maintainer='akdommers',
    maintainer_email='akdommers@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ackermann_to_twist = ackermann_to_twist.ackermann_to_twist:main',
            'heading_to_angle = ackermann_to_twist.heading_to_angle:main',
        ],
    },
)
