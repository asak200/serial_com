from setuptools import find_packages, setup

package_name = 'serial_com'

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
    maintainer='asak',
    maintainer_email='abdoorahmansk2004@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_com = serial_com.serial_com_server:main',
            'joint_broad = serial_com.joint_broad:main',
            'diff_cont = serial_com.diff_cont:main',
        ],
    },
)
