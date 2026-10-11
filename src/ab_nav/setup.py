import os
from glob import glob
from setuptools import setup

pkg = 'ab_nav'
setup(
    name=pkg, version='0.1.0', packages=[pkg],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + pkg]),
        ('share/' + pkg, ['package.xml']),
        (os.path.join('share', pkg, 'launch'), glob('launch/*.py')),
        (os.path.join('share', pkg, 'urdf'), glob('urdf/*')),
        (os.path.join('share', pkg, 'worlds'), glob('worlds/*')),
        (os.path.join('share', pkg, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Sasha', maintainer_email='alya.fedora@list.ru',
    description='A to B navigation with GPS waypoints', license='MIT',
    entry_points={'console_scripts': ['gps_waypoint_nav = ab_nav.gps_waypoint_nav:main']},
)
