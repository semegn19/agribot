from setuptools import find_packages, setup

package_name = 'agribot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='semegn',
    maintainer_email='semegn@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'navigation_node = agribot_navigation.navigation_node:main',
            'safety_stop = agribot_navigation.safety_stop:main',
            'stop_on_disease = agribot_navigation.stop_on_disease:main',
            'slam_node = agribot_navigation.slam_node:main',
            'odom_tf_bridge = agribot_navigation.odom_tf_bridge:main',
            'reactive_avoid = agribot_navigation.reactive_avoidance:main',
            'navigator_goal = agribot_navigation.navigator_goal:main',

        ],
    },
)
