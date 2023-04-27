# stlOptimalBoundingBox_CGAL
This is a short program that uses the CGAL optimal bounding box utility to define the bounding box of an STL.\
It saves the box.stl and a mesh.txt with the name of the mesh, center and the three axis along which the box developes.\

> This was compiled with CGAL-5.5.1\
>Older version of CGAL may not have the #include <CGAL/optimal_bounding_box.h>

# How to compile:
```bash
mkdir build
cd build 
cmake .. 
cmake --build .
```
