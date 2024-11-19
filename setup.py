from setuptools import setup

package_name = 'dataCollect'

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
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'collector = dataCollect.data_collect:main',
            'grapher = dataCollect.control:main',
            'computer = dataCollect.heavy_computation:main',
            'primer = dataCollect.prime_number:main',
            'mandelbroter = dataCollect.mandelbrot:main'
        ],
    },
)