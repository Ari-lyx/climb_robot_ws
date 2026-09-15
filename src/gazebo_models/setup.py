from setuptools import setup
setup(name='gazebo_models', version='0.1.0', packages=['gazebo_models'],
      data_files=[('share/ament_index/resource_index/packages',['resource/gazebo_models']),('share/gazebo_models',['package.xml'])],
      install_requires=['setuptools'], zip_safe=True, maintainer='Climb Robot Maintainers', maintainer_email='maintainer@example.com', description='Procedural tank worlds', license='Apache-2.0')
