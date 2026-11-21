from setuptools import setup

package_name = 'scensim_batch'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    extras_require={'torch': ['torch']},
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='maintainer@example.com',
    description='The batch executor (roadmap 092 K2): compiled scenarios over N slots',
    license='Apache-2.0',
    tests_require=['pytest'],
)
