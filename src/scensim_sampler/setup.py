from setuptools import setup

package_name = 'scensim_sampler'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.plugins'],
    package_data={package_name: ['schema/*.json']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='maintainer@example.com',
    description='OpenSCENARIO distributions to sample records (pure Python)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['scensim-sample = scensim_sampler.cli:main'],
        'scensim.sampler': [
            'independent = scensim_sampler.plugins.independent:from_content',
            'joint = scensim_sampler.plugins.joint:Joint',
            'importance = scensim_sampler.plugins.importance:Importance',
            'twise = scensim_sampler.plugins.twise:TWise',
        ],
    },
)
