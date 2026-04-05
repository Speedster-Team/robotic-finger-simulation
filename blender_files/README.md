# Onshape URDF Export with Blender
* RDS Speedster team
* Spring 2026
#### Credit: Powerhouse team - Cole Abbott, Heinrich Asbury, Jared Berry, Evan Bulatek, and Benji Sobeloff-Gittes

## Description
This method (created by the powerhouse team) can import onshape cad into a urdf for RDS robotic fingers.

No modifications to the original Onshape CAD were required for this approach. All elements were grouped into one of the 5 links.

## Instructions
This is a method that uses Blender to generate a URDF from a robot assembly created in Onshape.

1. Export full assembly from onshape as STL file
2. Import as STL to blender
3. Click on full STL -> `Edit mode -> A -> P -> Separate by loose parts`
4. Use wireframe and join meshes `(Ctrl + J)` to chunk mesh into 5 links
    * Name them: `base_link, mcp_link, proximal_phalanx, middle_phalanx, distal_phalanx`
    * Put these into a collection titled `visual`
5. Ensure `base_link` is at (0,0,0)
6. Select each mesh, look at incoming joint axis.
    * Go to edit mode
    * Select opposite faces on shaft
    * `Shift + S -> 3D Cursor to selected`
    * Back to object mode -> `Object -> Origin to 3D Cursor`
    * `Shift + A -> Empty -> Arrow`
    * Align arrow with axis of rotation (do not translate, only rotate!)
    * NOTE: For middle_phalanx, look at outgoing joint axis of proximal_phalanx
7. Add the Empty objects to a collection named `joint_axes`
    * Name them: `mcp_splay, mcp_flex, pip_flex, dip_flex`
8. Duplicate visual meshes and move duplicates to `collision` collection
    * Change names to `col_<visual_mesh_name>`
9. For each visual mesh:
    * `Edit mode -> A -> X -> Limited dissolve`
10. For each collision mesh:
    * `Edit mode -> A -> Mesh -> Convex Hull -> X -> Limited dissolve`
11. Run script from scripting tab in blender
    * Make sure to set up paths and robot geometry in script

![](image/finger_rviz.png)
The visual geometry in rviz

![](image/col_rviz.png)
The collision geometry in rviz

## AI usage
AI was used heavily in writing the code to export from blender to urdf. 

I fed back errors to Claude until the export code was in a state where I could debug myself (origin misplacement or other visualization errors).


