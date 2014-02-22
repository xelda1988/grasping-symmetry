GraspSymmetrizer
================

This Software takes as INPUT:

* List of grasps with corresponding object identifier (XML)
* List of objects with corresponding global symmetries (XML)
* Configuration file for Robotic Hand with corresponding Hand symmetries (XML)

The Software produces as OUTPUT:

* List of grasps produced by symmetry operations (XML, simple TXT, GraspIt! compatible XML)

Dependencies:

* Eigen3
* tinyXML2

Comments:

* Grasps are defined as a sequence of end-effector poses and hand joints
* Grasps can be loaded and exportet from GraspIt! .XML files
   