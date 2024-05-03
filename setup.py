from setuptools import setup
package_name = 'hv_bridge_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'libs.Controllers.arm', 'libs.Controllers.drive',
        'libs.Controllers.gripper', 'libs.Resources.client',
        'libs.Resources.config', 'libs.Resources.listener',
        'libs.Resources.mp', 'libs.Resources.reader',
        'libs.Resources.utils', 'libs.Resources.writer',
        'libs.Sensors.gripper', 'libs.Sensors.ir',
        'libs.Sensors.lidar', 'libs.Sensors.odometry',
        'libs.Services.avoid', 'libs.Services.follow',
        'libs.Services.pick_target', 'scripts.hv_client',
        'scripts.main_node', 'libs.__init__',
        'libs.Controllers.__init__', 'libs.Resources.__init__',
        'libs.Sensors.__init__', 'libs.Services.__init__',
        ],
    install_requires=['setuptools'],
    author='Nicholas Shindler',
    author_email="shindler25@gmail.com",
    maintainer='Nicholas Shindler',
    maintainer_email="shindler25@gmail.com",
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: GPLv3 :: Stuff',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='The bridging package for ROS2',
    license='GPLv3',
    entry_points={
        'console_scripts': [
            'hv_bridge = scripts.main_node:main',
        ],
    },
)
