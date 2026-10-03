<!--
  Placeholder for STL mesh files. URDF currently uses simple box/cylinder
  geometry (verified correct kinematics + collision; visual quality is
  intentionally placeholder).
  TODO: replace with real STL meshes once mechanical design finalizes.
-->
# meshes/arm

Placeholder mesh directory. The URDF uses simple box and cylinder geometry
until mechanical design is finalized. Replacement workflow:

1. Drop `link1.stl`, `link2.stl`, `link3.stl`, `link4.stl` here.
2. Edit `src/rak_description/urdf/rak.urdf.xacro` to swap
   `<geometry><box .../></geometry>` for
   `<geometry><mesh filename="package://rak_description/meshes/arm/linkN.stl"/></geometry>`.
3. Rebuild + relaunch - TF and inertia are unchanged; only visuals improve.