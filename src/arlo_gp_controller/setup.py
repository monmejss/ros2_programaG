from setuptools import find_packages, setup

package_name = 'arlo_gp_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='monica',
    maintainer_email='mbetsabe02@gmail.com',
    description='Controlador genético de Arlo en ROS 2 con nodos Python y C++',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = arlo_gp_controller.main:main',
        ],
    },
)
