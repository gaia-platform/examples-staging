from setuptools import setup

package_name = 'adc'

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
    maintainer='gaia',
    maintainer_email='steve@gaiaplatform.io',
    description='ADC nodes for analog sensors',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensors = adc.sensors:main',
        ],
    },
)
