from setuptools import find_packages, setup
# 配置具有相同特点的launch file
from glob import glob
package_name = 'py01_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/' + package_name, ['launch/py01_helloworld_launch.py'])
        # *_launch.py 为后缀的一并安装
        ('share/' + package_name, glob("launch/*_launch.py"))
        # ('share/' + package_name, [glob("launch/*_launch.xml")]),
        # ('share/' + package_name, [glob("launch/*_launch.yaml")])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tao',
    maintainer_email='dylanzhou415@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
