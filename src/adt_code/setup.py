from setuptools import find_packages, setup

package_name = 'adt_code'

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
    maintainer_email='daniel.nussbaum@htlstp.at',
    description='This package contains code from the ADT.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_bridge = adt_code.arduino_bridge:main',
        ],
    },
)
