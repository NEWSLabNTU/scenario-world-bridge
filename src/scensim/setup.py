from setuptools import setup

package_name = 'scensim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    # `project new`'s starters, read from beside scensim/project.py
    package_data={package_name: ['templates/*/*', 'templates/*/*/*', 'autoware_env.bash', 'overlay_env.bash']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # <prefix>/bin/scensim: the one command, on PATH after sourcing (081 step 3)
        ('bin', ['bin/scensim']),
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
            'project = scensim.project:main',
            'assets = scensim.assets:main',
            'seat = scensim.seat:main',
            'run = scensim.run:cli',
            'runs = scensim.runs:main',
            'isaac = scensim.isaac:main',
            'drive = scensim.drive:main',
            'autoware = scensim.autoware:main',
            'testbench = scensim.testbench_cli:main',
            'stack = scensim.stack_cli:main',
            'scensim = scensim.cli:main',
        ],
    },
)
