from setuptools import find_packages, setup

package_name = 'trigger_topic_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=['trigger_topic_pub'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs'],
    zip_safe=True,
    maintainer='canlab',
    maintainer_email='kcs@can-lab.co.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trigger_topic_pub = trigger_topic_pub:main',
        ],
    },
)
