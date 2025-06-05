from setuptools import find_packages, setup

package_name = 'tp_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tp2_e1.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='guirespi',
    maintainer_email='guidoramirez7@gmail.com',
    description='Package which complete practical works during FROB subject',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'srv = tp_pkg.tp2_e1_count_srv:main',
            'client = tp_pkg.tp2_e1_count_client:main'
        ]
    },
)
