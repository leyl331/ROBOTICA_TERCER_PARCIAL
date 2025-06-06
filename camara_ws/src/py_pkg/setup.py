from setuptools import find_packages, setup

package_name = 'py_pkg'

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
    maintainer='leyla',
    maintainer_email='lipaleyla9@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'sub = py_pkg.image_sub:main',
            'cal = py_pkg.calibrar:main',
            'ki = py_pkg.kinect:main',
            'kd = py_pkg.kinect_d:main',

            'im_prof = py_pkg.im_prof:main',
            'lidar = py_pkg.lidar:main',
             

            'ki_aruco= py_pkg.kinect_aruco:main',


            'vel= py_pkg.vel:main',
            'p3_e2= py_pkg.p3_e2:main',
            'dist_vel= py_pkg.dist_vel:main',

        ],
    },
)
