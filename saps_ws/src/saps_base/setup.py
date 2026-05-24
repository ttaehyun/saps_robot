from setuptools import find_packages, setup

package_name = 'saps_base'

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
    maintainer='a',
    maintainer_email='a@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mecanum_base_node = saps_base.mecanumBaseNode:main',
            'mpu6050_node = saps_base.mpu6050_node:main',
            'uwb_receiver_node = saps_base.uwb_receiver_node:main',
        ],
    },
)
