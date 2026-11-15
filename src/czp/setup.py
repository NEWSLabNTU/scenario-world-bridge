from setuptools import setup

package_name = 'czp'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_data={package_name: ['config/*.yaml', 'config/conventions/*.yaml']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=False,
    maintainer='Maintainer',
    maintainer_email='maintainer@example.com',
    description='The conflict-zone protocol (roadmap 078)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'czp-zones = czp.zones_tool:main',
            'interaction_strategy = czp.node:main',
        ],
    },
)
