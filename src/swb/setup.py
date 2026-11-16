from setuptools import setup

package_name = 'swb'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    # `project new`'s starters, read from beside swb/project.py
    package_data={package_name: ['templates/*/*', 'templates/*/*/*', 'autoware_env.bash', 'overlay_env.bash']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Maintainer',
    maintainer_email='maintainer@example.com',
    description="scenario-world-bridge's user commands",
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'project = swb.project:main',
            'assets = swb.assets:main',
            'seat = swb.seat:main',
            'run = swb.run:cli',
            'runs = swb.runs:main',
            'isaac = swb.isaac:main',
            'drive = swb.drive:main',
        ],
    },
)
