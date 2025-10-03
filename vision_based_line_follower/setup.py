from setuptools import find_packages, setup

package_name = 'vision_based_line_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial', 'opencv-python'],
    zip_safe=True,
    maintainer='yash_soni_in11',
    maintainer_email='soni.yash.official@gmail.om',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_node = line_follower_nodes.camera_node:main',
            'line_detector = line_follower_nodes.line_detector:main',
            'controller_node = line_follower_nodes.controller_node:main',
        ],
    },
)
