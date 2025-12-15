from setuptools import find_packages, setup

package_name = 'phantomx_pincher_opencv'

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
    maintainer='johotan',
    maintainer_email='johalopezari@unal.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pxp_camera_publisher_node = phantomx_pincher_opencv.pxpCameraPublisher:main',
            'pxp_camera_subscriber_node = phantomx_pincher_opencv.pxpCameraSubscriber:main',
        ],
    },
)
