
#!/usr/bin/env python
import distutils
from distutils.core import setup

setup(name = "rrt",
      version = '0.5',
      description = "Simple implementation of RRT and RRT*",
      author = "Kevin A Smith",
      author_email= "k2smith@mit.edu",
      url = "https://github.com/kasmith/rrt",
      packages = ["rrt", "rrt.viz", "rrt.interface"],
      requires = ["numpy", "pygame", "geometry"],
      suggests = ["phystables"])
