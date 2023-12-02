from setuptools import find_packages, setup

package_name = 'adt'

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
    maintainer='Daniel Nussbaum',
    maintainer_email='daniel.nussbaum@nussi.net',
    description='Dieses packet is vertantwortlich für die Steuerung des ADT',
    license='MIT License (File in adt/LICENSE)',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'base_drive_serial_bridge = adt.base_drive_serial_bridge:main',
                'base_vision_dosen_detector = adt.base_vision_dosen_detector:main',
        ],
    },
)
