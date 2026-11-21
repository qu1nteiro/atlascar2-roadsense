from glob import glob

from setuptools import setup

package_name = 'va_g03'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Group 03',
    maintainer_email='grupo03@ua.pt',
    description='Group 03: simulated driver and perception publishing the local occupancy grid',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fake_scan = va_g03.fake_scan:main',
            'scan_to_grid = va_g03.scan_to_grid:main',
        ],
    },
)
