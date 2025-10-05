from setuptools import find_packages, setup

package_name = 'auv_gate_nav'

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
    maintainer='SudhishnaJanavi',
    maintainer_email='sudhishna.janavi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_features = auv_gate_nav.safety_features:main',
            'obstacle_avoidance = auv_gate_nav.obstacle_avoidance:main',
            'autonomous_auv_controller = auv_gate_nav.autonomous_auv_controller:main',
        ],
    },
)
