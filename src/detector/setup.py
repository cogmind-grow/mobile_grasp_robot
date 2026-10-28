import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
            'console_scripts': [
                'bottle_detector_node = detector.bottle_detector:main',
                'pointcloud_test_node = pointcloud_test.pointcloud_test:main',
                'pointcloud_tf_node = pointcloud_test.pointcloud_tf:main',
                'bottle_localizer_node = detector.bottle_localizer:main',
            ],
        },
)
