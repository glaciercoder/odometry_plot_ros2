from setuptools import find_packages, setup
import glob
import os

package_name = 'odometry_processing'

odom_data_files = []
for root, _, files in os.walk('data'):
    for file in files:
        file_path = os.path.join(root, file)
        # Append (target_directory, source_file) for each file
        target_path = os.path.relpath(root, '.')  # Preserve relative directory structure
        odom_data_files.append((os.path.join('share', package_name, target_path), [file_path]))
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/odom_plot.launch.py']),
        ('share/' + package_name, ['odometry_processing/odom_plot.py']),
        *odom_data_files,
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dssl',
    maintainer_email='weibingchuan20@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
