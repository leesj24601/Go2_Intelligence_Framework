from setuptools import find_packages, setup


package_name = "go2_gui_controller"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/waypoints.yaml"]),
        (f"share/{package_name}/launch", ["launch/go2_gui_controller.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cvr",
    maintainer_email="cvr@local",
    description="GUI controller for Go2 navigation with Nav2 and manual control.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "gui_controller = go2_gui_controller.main:main",
        ],
    },
)
