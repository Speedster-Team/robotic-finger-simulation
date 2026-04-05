from setuptools import find_packages, setup

package_name = 'finger_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/basic_fingersim.launch.xml']),
        ('share/' + package_name + '/config', ['config/base_sim.rviz']),
        
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mjenz',
    maintainer_email='michael.jenz77@gmail.com',
    description='Simulate robotic finger in Drake',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'basicsim = finger_simulation.basic_finger_sim:main',
        ],
    },
)
