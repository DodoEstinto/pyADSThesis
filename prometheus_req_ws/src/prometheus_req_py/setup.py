from setuptools import find_packages, setup

package_name = 'prometheus_req_py'

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
    maintainer='Davide Paossi',
    maintainer_email='davidepaossi@gmail.com',
    description='Package containing nodes that implements Equipment Requests call',
    license='Modified MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ads_node = prometheus_req_py.ADS.ads_node:main',
            'client = prometheus_req_py.Client.GUI_node:main',
            'client_API = prometheus_req_py.Client.API_node:main'
        ],
    },
)
