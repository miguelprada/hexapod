# hexapod_description package

This packages contains the model of the hexapod robot.

[`leg.urdf.xacro`](model/leg.urdf.xacro) defines the geometrical and visual<sup>*</sup> model of a single leg. The gazebo-specific components are described separately in [`leg.gazebo.xacro`](model/leg.gazebo.xacro), and the transmissions required by `gazebo_ros_control` are included in [`leg.transmission.xacro`](leg.transmission.xacro).

---

\* The meshes being used in the model were obtained from [this ROBOTIS support page][robotis_cad_page].

[robotis_cad_page]: http://en.robotis.com/BlueAD/board.php?bbs_id=downloads&mode=view&bbs_no=26324&page=1&key=&keyword=&sort=&scate=
