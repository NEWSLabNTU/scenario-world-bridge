"""scensim's pip front end: installs the prebuilt scensim for this host's ROS 2 and runs it.

The module is not called ``scensim``: the prebuilt release puts its own ``scensim`` package
on ``PYTHONPATH`` (which precedes site-packages), and would shadow it.
"""

__version__ = "0.2.1"
