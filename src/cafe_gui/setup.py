from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'cafe_gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, f'{package_name}.widgets'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'ui'), glob('ui/*.ui')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muammer',
    maintainer_email='muammer@example.com',
    description='GUI for autonomous cafe serving robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_node = cafe_gui.gui_node:main',
            'test_robot_node = cafe_gui.test_robot_node:main',  # Ekleyin
        ],
    },
)