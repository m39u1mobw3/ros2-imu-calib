from setuptools import setup

package_name = "imu_calib"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/calibrate.launch.py"]),
        (f"share/{package_name}/config", ["config/imu_calib.yaml"]),
    ],
    install_requires=["setuptools", "numpy"],
    entry_points={"console_scripts": ["calibrate = imu_calib.calibrate_node:main"]},
)
