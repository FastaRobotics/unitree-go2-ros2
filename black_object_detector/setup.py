from setuptools import find_packages, setup

package_name = 'black_object_detector'

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
    maintainer='roboticlab1',
    maintainer_email='erfantabatabaei1380@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_black_objects = black_object_detector.detect_black_objects:main',
            'move_gazebo_object = black_object_detector.give_move_to_object:main'

        ],
    },
)
