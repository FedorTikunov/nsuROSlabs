from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'carrot_rotate'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hotinfdensedot',
    maintainer_email='fedor.tikunov@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'static_turtle_tf2_broadcaster = carrot_rotate.static_turtle_tf2_broadcaster:main',
        'turtle_tf2_broadcaster = carrot_rotate.turtle_tf2_broadcaster:main',
        'turtle_tf2_listener = carrot_rotate.turtle_tf2_listener:main',
        'turtle_carrot = carrot_rotate.turtle_carrot:main',
        ],
    },
)
