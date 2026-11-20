from glob import glob

from setuptools import setup

package_name = 'va_g00'

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
    maintainer='Grupo 00',
    maintainer_email='grupo00@ua.pt',
    description='Grupo 00: driver (simulado) e perceção que publica a grelha local',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fake_scan = va_g00.fake_scan:main',
            'scan_to_grid = va_g00.scan_to_grid:main',
        ],
    },
)
