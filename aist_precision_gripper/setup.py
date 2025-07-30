from glob       import glob
from setuptools import find_packages, setup


package_name = 'aist_precision_gripper'

setup(name=package_name,
      version='0.0.0',
      packages=find_packages(),
      data_files=[
          ('share/ament_index/resource_index/packages',
           ['resource/' + package_name]),
          ('share/' + package_name, ['package.xml']),
          ('share/' + package_name + '/launch', glob('launch/*')),
          ('share/' + package_name + '/config', glob('config/*')),
          ('share/' + package_name + '/urdf',   glob('urdf/*')),
          ('share/' + package_name + '/meshes/visual',
           glob('meshes/visual/*')),
          ('share/' + package_name + '/meshes/collision',
           glob('meshes/collision/*'))
      ],
      install_requires=['setuptools'],
      zip_safe=True,
      maintainer='Toshio Ueshiba',
      maintainer_email='t.ueshiba@aist.go.jp',
      description='Controller for precision gripper',
      license='BSD',
      tests_require=['pytest'],
      entry_points={
          'console_scripts':
          ['precision_gripper_controller = aist_precision_gripper.precision_gripper_controller:main',
           'test_gripper_client = aist_precision_gripper.test_gripper_client:main']
      }
)
