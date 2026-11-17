from setuptools import setup

setup(name="a_support_pkg", version="0.0.1", packages=["a_support_pkg"],
      data_files=[("share/ament_index/resource_index/packages", ["resource/a_support_pkg"]),
                  ("share/a_support_pkg", ["package.xml"])],
      entry_points={"console_scripts": ["pilot = a_support_pkg.pilot:main"]})
