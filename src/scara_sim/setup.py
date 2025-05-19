import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'scara_sim'
shared_directories = [
    'launch',
    'meshes',
    'description',
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *list(map(lambda d: (f'share/{package_name}/{d}', glob(os.path.join(d, '*'))), shared_directories))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lopatoj',
    maintainer_email='justin@lopato.org',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_node = scara_sim.sim_node:main'
        ],
    },
)
