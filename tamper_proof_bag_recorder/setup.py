import os
from glob import glob
from setuptools import setup

package_name = 'tamper_proof_bag_recorder'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laura',
    maintainer_email='laura@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tp_bag_recorder_srv = tamper_proof_bag_recorder.tp_bag_recorder_srv:main',
            'proof_checker_srv = tamper_proof_bag_recorder.proof_checker_srv:main',
            'contract_deployment = tamper_proof_bag_recorder.contract_deployment:main',
            'bag_reader_proof = tamper_proof_bag_recorder.bag_reader_proof:main'
        ],
    },
)
