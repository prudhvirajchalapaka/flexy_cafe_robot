from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'flexy_butler'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prudhviraj',
    maintainer_email='prudhvirajchalapaka@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task1=flexy_butler.task1:main',
            'task2=flexy_butler.task2:main',
            'task3=flexy_butler.task3:main',
            'task4=flexy_butler.task4:main',
            'task5=flexy_butler.task5:main',
            'task6=flexy_butler.task6:main',
            'task7=flexy_butler.task7:main',
            'final_task=flexy_butler.final_butlerr:main',

        ],
    },
)
