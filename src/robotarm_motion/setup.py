from setuptools import find_packages, setup

package_name = 'robotarm_motion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Samuel Wuetherich',
    maintainer_email='Samuel.Wuetherich@t-online.de',
    description='MoveIt motions for the robotarm, including the control_center mission handshake',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'demo_motions = robotarm_motion.demo_motions:main',
        ],
    },
)
