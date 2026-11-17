from glob import glob

from setuptools import setup

setup(
    name="rammp_drinking",
    version="0.1.2",
    packages=["rammp_drinking"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/rammp_drinking"]),
        ("share/rammp_drinking", ["package.xml"]),
        ("share/rammp_drinking/launch", glob("launch/*.py")),
        ("share/rammp_drinking/config", glob("config/*.yaml")),
        ("share/rammp_drinking/config/machines", glob("config/machines/*.yaml")),
    ],
    install_requires=["setuptools"],
    entry_points={"console_scripts": ["rammp-drink = rammp_drinking.cli:main",
                                      "drinking_node = rammp_drinking.engine:main"]},
)
