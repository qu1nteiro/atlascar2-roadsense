from setuptools import setup

package_name = 'va_planeamento'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='VA teaching staff',
    maintainer_email='va@ua.pt',
    description='Local occupancy grid and local A* planner',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'local_astar = va_planeamento.local_astar:main',
        ],
    },
)
