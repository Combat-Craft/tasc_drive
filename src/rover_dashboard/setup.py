from setuptools import setup

package_name = 'rover_dashboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ahmed',
    maintainer_email='you@example.com',
    description='Custom PyQt ROS 2 rover dashboard with hardware relay and motor telemetry backend',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dashboard = rover_dashboard.dashboard:main',
            'hardware_backend = rover_dashboard.hardware_backend:main',
        ],
    },
)
