from glob import glob

from setuptools import find_packages, setup

package_name = 'airmouse'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mvn',
    maintainer_email='mvn@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fly_square = airmouse.fly_square:main',
            'slam_to_mavros = airmouse.slam_to_mavros:main',
            'grid_mapper = airmouse.grid_mapper:main',
            'explorer = airmouse.explorer:main',
            'dashboard_server = airmouse.dashboard_server:main',
            'survivor_tagger = airmouse.survivor_tagger:main',
        ],
    },
)
