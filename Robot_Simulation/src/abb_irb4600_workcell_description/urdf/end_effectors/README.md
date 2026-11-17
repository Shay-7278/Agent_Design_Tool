# SCHUNK PGF-100 simulation tool

The active macro is schunk_pgf_100_gripper_macro.xacro. tools.yaml is the single
source of the flange-to-TCP transform and mesh references. This is a direct
mount: no QC master. CAD coupling face z=41.78 mm maps to the flange; pad midpoint
z=-201.35 mm maps to flange +243.13 mm. Mesh orientation is Rx(pi).
The published frame is LL_schunk_tcp. ROS adapter quaternions are xyzw.

Both finger joints receive equal positive closing travel of 8.875 mm. Their
CAD axes are opposite X directions. Simulated joint bounds include a 1 mm
numerical margin; commanded stroke is unchanged. DetachableJoint supplies
deterministic transport, not proof of a frictional grasp.

Original STL/CAD exports remain under visual/ and collision/. Runtime meshes
are under meshes/optimized/end_effectors with a geometry validation report.
Inertias remain provisional. The nominal/calibrated TCP pair, TC-master chain,
workcell_api and MoveIt references from the previous README are retired.
Real TCP calibration and RAPID execution remain Phase II.
