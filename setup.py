import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'models/Jetbot_v1'), glob('models/Jetbot_v1/**/*', recursive=True)),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        ('share/' + package_name, ['package.xml']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hubert',
    maintainer_email='hubert@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello = my_package.hello:main',
            'jetbot_spawn = my_package.jetbot_spawn:main',
            'jetbot_search_random = my_package.jetbot_search_random:main',
            'jetbot_search_grid = my_package.jetbot_search_grid:main',
            'jetbot_search_random_separate = my_package.jetbot_search_random_separate:main',
            'jetbot_detect = my_package.jetbot_detect:main',
            'jetbot_pather = my_package.jetbot_pather:main',
            'head = my_package.head:main'
        ],
    },
)
