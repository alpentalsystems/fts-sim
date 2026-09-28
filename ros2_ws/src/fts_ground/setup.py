from setuptools import setup

PACKAGE = "fts_ground"

setup(
    name=PACKAGE,
    version="0.1.0",
    packages=[PACKAGE],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE]),
        ("share/" + PACKAGE, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alpental Systems",
    maintainer_email="contact@alpentalsystems.com",
    description="Ground station and scenario runner for the simulated flight termination system.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ground_station = fts_ground.ground_station:main",
            "runner = fts_ground.runner:main",
        ],
    },
)
