import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'adt_code'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sans',
    maintainer_email='sans@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'basis_roboter_demo = adt_code.basis_roboter_demo:main'
            'base_roboter_detection = adt_code.base_roboter_detection:main'
        ],
    },
)
