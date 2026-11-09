from setuptools import setup
import os
from glob import glob

pkg = 'agv_tag_nav'
setup(
    name=pkg, version='0.1.0', packages=[pkg],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + pkg]),
        ('share/' + pkg, ['package.xml']),
        (os.path.join('share', pkg, 'launch'), glob('launch/*.py')),
        (os.path.join('share', pkg, 'config'), glob('config/*.yaml')),
        (os.path.join('share', pkg, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    entry_points={'console_scripts': [
        'tag_map_node = agv_tag_nav.tag_map_node:main',
        'tag_detector = agv_tag_nav.tag_detector:main',
        'tag_localizer = agv_tag_nav.tag_localizer:main',
        'tag_navigator = agv_tag_nav.tag_navigator:main',
        'safety_stop = agv_tag_nav.safety_stop:main',
    ]},
)
