from setuptools import find_packages, setup

package_name = 'prometheus_req_py'
bringup_package_name= 'prometheus_req_bringup'

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
    maintainer='ubuntu',
    maintainer_email='enrico.piccinini@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'equipment_state_pub = prometheus_req_py.equipment_state_pub:main',
            'equipment_state_sub = prometheus_req_py.equipment_state_sub:main'
        ],
    },
)
