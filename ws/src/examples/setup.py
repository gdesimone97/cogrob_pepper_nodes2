from setuptools import find_packages, setup

package_name = 'examples'

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
    maintainer='mivia',
    maintainer_email='44608428+gdesimone97@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "head_motion_example=examples.head_motion_example:main",
            "tablet_example=examples.tablet_example:main",
            "tts_example=examples.tts_example:main",
            "wakeup_example=examples.wakeup_example:main",
            
        ],
    },
)
