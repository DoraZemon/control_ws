from setuptools import find_packages, setup

package_name = 'my_arm_coupling'

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
    maintainer='zsz',
    maintainer_email='zsz@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	'joint_coupler = my_arm_coupling.joint_coupler:main',
        	'display_trajectory_coupler = my_arm_coupling.display_trajectory_coupler:main',
        	'joint_state_filter = my_arm_coupling.joint_state_filter:main',
        ],
    },
)
