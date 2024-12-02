from setuptools import find_packages, setup

package_name = 'control_arm'

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
    maintainer='takeoff',
    maintainer_email='vinayakkapoor12@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'end_effector = control_arm.forward_kinematics:main',
            'inverse_kinematics = control_arm.inverse_kinematics:main',
<<<<<<< HEAD
=======
            'velocity_kinematics = control_arm.velocity_kinematics:main',
>>>>>>> 55be825 (Added velocity node Assigment 2 part 1)
            'move_client = control_arm.move_client:main',
            'pickup = control_arm.pickup:main'
        ],
    },
)
