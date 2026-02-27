from setuptools import find_packages, setup

package_name = 'sauvc26_code'

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
    maintainer='aliy',
    maintainer_email='fawwasaliy11@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "test = sauvc26_code.test:main",
            "move = sauvc26_code.move:main",
            "arm = sauvc26_code.arm:main"
        ],
    },
)
