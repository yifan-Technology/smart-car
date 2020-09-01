from setuptools import setup

package_name = 'yfcam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    scripts=[(package_name+"/yf_node.py"),
		(package_name+"/yf_costmap.py"),
		(package_name+"/yf_humanDetect.py"),
		(package_name+"/yf_pattenRecognition.py"),
		(package_name+"/yf_camera.py"),
		(package_name+"/motorSerial.py"),
		(package_name+"/dwa_module.py")
		],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hzs',
    maintainer_email='hzs@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'leftcam = yfcam.yf_camera:main',
		'motorSerial = yfcam.motorSerial:main',
		'dwaControll = yfcam.yf_dwa:main',
        ],
    },
)
