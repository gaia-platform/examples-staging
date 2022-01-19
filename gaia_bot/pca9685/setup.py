from setuptools import setup

package_name = 'pca9685'

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
    maintainer='gaiabot',
    maintainer_email='steve@gaiaplatform.io',
    description='Nodes making use of pca9685 interface',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'neck_pose = pca9685.neck_pose:main',
            'wheels = pca9685.wheels:main',
        ],
    },
)
