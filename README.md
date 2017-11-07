ay_py
==================
Python libraries for robot learning, including optimization, Graph-DDP, locally weighted regression, neural networks for regression and classification, geometry and kinematics calculation, robot control interface of Baxter, PR2, Robotiq, ROS utility, etc.


Author
==================
Akihiko Yamaguchi, http://akihikoy.net/


Acknowledgment
==================
CMA-ES (src/ay_py/thirdp/cma.py) is implemented by Nikolaus Hansen.  Read src/ay_py/thirdp/CMA1.0.09-README.txt for more information.


Requirements
==================

ay_py.core
------------------------
- Python: core, numpy, scipy

ay_py.ros
------------------------
Binaries are available (you can use **apt-get**):
- ROS core system, rospy, roscpp, std_msgs, std_srvs, geometry_msgs, tf, ...
- ros-ROS_DISTR-moveit-full  (ROS_DISTR: groovy, hydro, indigo, etc.)
- ros-ROS_DISTR-moveit-resources

Working with Baxter/PR2 (OPTIONAL):
- Need SDK packages for Baxter/PR2
- Information would be found in: http://akihikoy.net/notes/?text


Usage
==================
Add `src/ay_py/` to Python package path.  Under `ay_py`, there are sub modules:

```
import sys
sys.path.append('YOUR_DIRECTORY/ay_py/src')
```

ay_py.core
------------------------
ROS-independent module, including optimization, Graph-DDP, locally weighted regression, neural networks for regression and classification, geometry and kinematics calculation, etc.  You can use this module WITHOUT ROS.  Just import the package:

```
from ay_py.core import *
```

ay_py.thirdp
------------------------
Packages from third party are stored.  They are internally used in ay_py.*.

ay_py.ros
------------------------
ROS-dependent module, including utility, robot control interface of Baxter, PR2, Robotiq, etc.  ROS environment should be setup before using this module.

```
from ay_py.ros import *
```

ay_py.tools
------------------------
Useful tools, such as GUI with Python.  For example, run:

```
python src/ay_py/tool/py_gui.py
```

demo
------------------------
In `demo` directory, many demonstration scripts using `ay_py.core` are contained.  Directly run them.

demo_ros
------------------------
In `demo_ros` directory, some demonstration scripts using `ay_py.ros` are contained.  Directly run them.


Troubles
==================
Send e-mails to the author.

