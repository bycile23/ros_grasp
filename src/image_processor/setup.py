from setuptools import find_packages, setup

package_name = 'image_processor'

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
    maintainer='baitiao',
    maintainer_email='baitiao@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 格式：'可执行名 = 包名.文件名:main函数'
            'image_converter = image_processor.image_converter_node:main',
            'vision_manager = image_processor.vision_manager_node:main',
        ],
    },
)
