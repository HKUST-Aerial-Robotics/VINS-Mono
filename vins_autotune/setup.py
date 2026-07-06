from setuptools import setup

package_name = "vins_autotune"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/autotune.launch.py"]),
        ("share/" + package_name + "/config", ["config/autotune_params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="innospector",
    maintainer_email="user@innospector.local",
    description="Online parameter auto-tuning for VINS-Mono using ground truth odometry",
    license="GPLv3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vins_autotune_node = vins_autotune.autotune_node:main",
            "vins_autotune_offline = vins_autotune.offline_tuner:main",
        ],
    },
)