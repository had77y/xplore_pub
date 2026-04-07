from setuptools import setup

package_name = 'rover_xplore_pub'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='had77y',
    maintainer_email='hady.azzy@epfl.ch',
    description='Rover XPlore — nœuds côté client',
    license='MIT',
    entry_points={
        'console_scripts': [
            'video_viewer_node = rover_xplore_pub.video_viewer_node:main',
        ],
    },
)