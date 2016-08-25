# Body design

My original plan was to build the hexa(tetra)pod following the instructions on the Robotis spider robot manual. However, I recently realized that the required _F51_ and _F52_ parts are missing from my kit. Therefore, I've decided to design a custom body for my hexa(tetra)pod.

The first approach I thought of was to 3D print a custom body, but I was concerned about the quality and precision of the required mounting holes for mounting the legs using _F3_ brackets. Finally, I've decided to use two laser-cut plastic plates to which I will bolt _F3_ brackets to match those which should be mounted on the bottom of the _AX_ servos. The image below shows more or less how this should end up.

**ToDo: put a picture of a tentative version of the hexa(tetra)pod**

## Body dimensions

In order to settle into specific body dimensions, I wanted to be able to automatically visualize several possibilities. To do this I figured I should parametrize the shape of the body and generate the models for visualization (and hopefully fabrication, once the parameters are chosen) automatically from a given set of parameters.

Process (should all be done by a single script):
- Use OpenSCAD to generate the plates stl from a given parameter combination
    + Since OpenSCAD exports ASCII stl and rviz requires binary stl, the `admesh` utility should be usable to convert automatically
    + If this is not possible, it can also be done with FreeCAD, although I'm not sure if I can automate this or I will need to do it manually though the GUI
- Generate the xacro with the newly generated plates and the correct leg placement
- [optional] open rviz to visualize the result and save an image?
