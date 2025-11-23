import os
from setuptools import setup, find_packages
from glob import glob

package_name = 'flexy_webdashboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        # Include web files
        (os.path.join('share', package_name, 'web'), 
            glob('web/*.*')),
        (os.path.join('share', package_name, 'web/css'), 
            glob('web/css/*') if os.path.exists('web/css') else []),
        (os.path.join('share', package_name, 'web/js'), 
            glob('web/js/*') if os.path.exists('web/js') else []),
    ],
    install_requires=[
        'setuptools',
        'flask',
        'flask-socketio',
    ],
    zip_safe=True,
    maintainer='prudhviraj',
    maintainer_email='your_email@example.com',
    description='Web dashboard for Flexy robot control and monitoring',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webdashboard_node = flexy_webdashboard.web_serverr:main',
        ],
    },
)