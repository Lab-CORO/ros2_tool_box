from setuptools import find_packages, setup

package_name = 'camera_calibration'

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
    maintainer='saxtot',
    maintainer_email='saxtot@gmail.com',
    description='Package to house camera calibration utilities',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'remap_node = camera_calibration.remap_node:main'
        ],
    },
)
