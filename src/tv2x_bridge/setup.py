from setuptools import find_packages, setup

package_name = 'tv2x_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/config.yaml']),
        ('share/' + package_name + '/launch', ['launch/tv2x_bridge_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Edoardo Torrini',
    maintainer_email='edoardo.torrini@gmail.com',
    description='V2x_bridge: bridge for the V2X communication',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tv2x_bridge = tv2x_bridge.tv2x_bridge:main'
        ],
    },
)
