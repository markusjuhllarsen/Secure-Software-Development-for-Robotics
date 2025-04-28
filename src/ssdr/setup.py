from setuptools import setup, find_packages

package_name = 'ssdr'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    py_modules=[],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'geometry_msgs',
        'cryptography',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', 
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
        ],
    zip_safe=True,
    maintainer='mjl',
    maintainer_email='markusjuhllarsen@gmail.com',
    description='Secure Software Development for Robotics',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = ssdr.main:main',
            'main_security = ssdr.main_security:main',
            'remote = ssdr.remote:main',
        ],
    },
)