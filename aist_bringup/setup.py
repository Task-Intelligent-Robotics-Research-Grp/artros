from setuptools import setup, find_packages
from glob import glob

package_name = "aist_bringup"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/config", glob("config/*.rviz")),
        ("share/" + package_name + "/config/templates", glob("config/templates/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/launch/inc", glob("launch/inc/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Toshio Ueshiba",
    maintainer_email="t.ueshiba@aist.go.jp",
    description="Package with bringup scripts/config for various robots",
    license="BSD",
    tests_require=["pytest"],
    entry_points={},
)
