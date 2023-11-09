from setuptools import setup

package_name = 'adt_web'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Nussbaum',
    maintainer_email='daniel.nussbaum@nussi.net',
    description='Sends data to the Website',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_provider = adt_web.data_provider:main'
        ],
    },
)
