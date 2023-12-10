from setuptools import setup

package_name = "center_mean"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="your-name",  # Replace with your name
    maintainer_email="your-email@example.com",  # Replace with your email
    description="Description of your package",  # Add a short description
    license="License of your package",  # Specify the license
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "center_mean_publisher = center_mean.center_mean_publisher:main",
        ],
    },
)
