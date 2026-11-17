from setuptools import setup

setup(name="fixture_world", version="0.0.1", packages=["fixture_world"],
      data_files=[("share/ament_index/resource_index/packages", ["resource/fixture_world"]),
                  ("share/fixture_world", ["package.xml"])],
      entry_points={"console_scripts": ["fake_world = fixture_world.server:main"]})
