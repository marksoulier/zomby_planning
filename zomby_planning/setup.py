from setuptools import find_packages, setup
import os
package_name = 'zomby_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[        
        (os.path.join('share', package_name, 'config'), ['config/config.yaml']),
        (os.path.join('share', package_name, 'config'), ['config/config_usu.yaml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yocto',
    maintainer_email='mark.soulier@usu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planner = zomby_planning.path_planner:main',
            'go_to_goal = zomby_planning.go_to_goal:main',
        ],
    },
)
