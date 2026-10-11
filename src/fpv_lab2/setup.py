from setuptools import find_packages, setup

package_name = 'fpv_lab2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='alina.koman.ir.2024@lpnu.ua',
    description='Lab 2: flight to a target point in ArduPilot SITL + Gazebo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # початковий приклад викладача: зліт - зависання - посадка
            'flight_test = flight_test.main:main',
            # власний алгоритм: політ до цільової точки (варіант 9)
            'fly_to_target = flight_test.target_main:main',
        ],
    },
)
