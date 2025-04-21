from setuptools import find_packages, setup
import os
package_name = 'qube_bringup'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/bringup.launch.py']),
        (os.path.join('share', package_name, 'urdf'), ['urdf/controlled_qube.urdf.xacro'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lhf',
    maintainer_email='liam.folsviks@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
