from setuptools import find_packages, setup

package_name = 'adt_cv'

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
    maintainer='adt',
    maintainer_email='daniel.nussbaum@htlstp.at',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detection = adt_cv.aruco_detection:main',
            'drink_detection = adt_cv.drink_detection:main'
        ],
    },
)
