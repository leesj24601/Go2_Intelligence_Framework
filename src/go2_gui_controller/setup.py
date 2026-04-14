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
        (
            f"share/{package_name}/launch",
            [
                "launch/go2_gui_controller.launch.py",
                "launch/go2_web_controller.launch.py",
            ],
        ),
        (f"share/{package_name}/web", ["go2_gui_controller/web/index.html"]),
    ],
    install_requires=[
        "setuptools",
        "PyYAML",
        "fastapi",
        "uvicorn[standard]",
    ],
    zip_safe=False,
    maintainer="cvr",
    maintainer_email="liberties24601@gmail.com",
    description="GUI controller for Go2 navigation with Nav2 and manual control.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "gui_controller = go2_gui_controller.main:main",
            "web_controller = go2_gui_controller.web_app:main",
            "odom_restamper = go2_gui_controller.odom_restamper:main",
            "image_restamper = go2_gui_controller.image_restamper:main",
            "camera_info_restamper = go2_gui_controller.camera_info_restamper:main",
            "rgbd_restamper = go2_gui_controller.rgbd_restamper:main",
            "rgbd_odom_sync = go2_gui_controller.rgbd_odom_sync:main",
        ],
    },
)
