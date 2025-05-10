from setuptools import find_packages, setup

package_name = 'quadruped_serial_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/quadruped_serial_interface.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyz',
    maintainer_email='1783866380@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'quadruped_serial_interface = quadruped_serial_interface.quadruped_serial_interface:main',
        ],
    },
)
