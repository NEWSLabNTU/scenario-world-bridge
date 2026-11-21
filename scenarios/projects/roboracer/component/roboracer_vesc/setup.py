from setuptools import setup

setup(
    name="roboracer_vesc",
    version="0.1.0",
    packages=["roboracer_vesc"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/roboracer_vesc"]),
                ("share/roboracer_vesc", ["package.xml"])],
    install_requires=["setuptools"],
    zip_safe=True,
    description="A stand-in RoboRacer drive unit for scensim roadmap 091 W23 (an example)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"scensim.vehicle_components": [
        "roboracer_vesc = roboracer_vesc.component:RoboRacerVesc"]},
)
