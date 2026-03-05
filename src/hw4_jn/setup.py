from setuptools import find_packages, setup

import os
import glob

package_name = 'hw4_jn'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'), glob.glob(os.path.join('config', '*.rviz'))),
        (os.path.join('share', package_name, 'config'), glob.glob(os.path.join('config', '*.*'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jn2',
    maintainer_email='jar3dnorth51@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "localization = hw4_jn.localization:main",
            "test_image = hw4_jn.test_image:main"
        ],
    },
)
