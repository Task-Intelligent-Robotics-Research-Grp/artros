from setuptools import setup, find_packages
from glob import glob

package_name = "aist_skills"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=['test']),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/launch", glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Toshio Ueshiba",
    maintainer_email="t.ueshiba@aist.go.jp",
    description="Package of actions for executing various tasks with robots",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        'console_scripts': [
            'interactive = ' + package_name + '.interactive:main'
        ],
    },
)
