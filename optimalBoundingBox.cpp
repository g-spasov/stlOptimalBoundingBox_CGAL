#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/optimal_bounding_box.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Real_timer.h>
#include <CGAL/Vector_3.h>
#include <CGAL/IO/STL.h>

#include <fstream>
#include <iostream>
namespace PMP = CGAL::Polygon_mesh_processing;
typedef CGAL::Exact_predicates_inexact_constructions_kernel    K;
typedef K::Point_3                                             Point;
typedef K::Vector_3                                            Vector;
typedef CGAL::Surface_mesh<Point>                              Surface_mesh;
int main(int argc, char** argv)
{
  const std::string filename = (argc > 1) ? argv[1] : CGAL::data_file_path("meshes/pig.stl");
  Surface_mesh sm;
  if(!PMP::IO::read_polygon_mesh(filename, sm) || sm.is_empty())
  {
    std::cerr << "Invalid input file." << std::endl;
    return EXIT_FAILURE;
  }
  CGAL::Real_timer timer;
  timer.start();
  // Compute the extreme points of the mesh, and then a tightly fitted oriented bounding box
  std::array<Point, 8> obb_points;
  CGAL::oriented_bounding_box(sm, obb_points,
                              CGAL::parameters::use_convex_hull(true));
  std::cout << "Elapsed time: " << timer.time() << std::endl;
  // Make a mesh out of the oriented bounding box
  Surface_mesh obb_sm;
  CGAL::make_hexahedron(obb_points[0], obb_points[1], obb_points[2], obb_points[3],
                        obb_points[4], obb_points[5], obb_points[6], obb_points[7], obb_sm);


  std::ofstream("obb.ply") << obb_sm;
  PMP::triangulate_faces(obb_sm);
  std::cout << "Volume: " << PMP::volume(obb_sm) << std::endl;

  //Let's save the stl file
  std::ofstream out("obb.stl");
  CGAL::IO::write_STL(out, obb_sm);
  out.close();



  Point center ={0,0,0};
  double_t x=0,y=0,z=0;

  for(int i=0;i<8;i++){
    x+=obb_points[i].x();
    y+=obb_points[i].y();
    z+=obb_points[i].z();
  }

  
  center = {
    x/8,
    y/8,
    z/8
  };

  std::cout << "Center: " << center << std::endl;
  Vector i(obb_points[0],obb_points[1]),j(obb_points[0],obb_points[3]),k(obb_points[0],obb_points[5]);

  std::cout << "i: " << i << std::endl;
  std::cout << "j: " << j << std::endl;
  std::cout << "k: " << k << std::endl;

  return EXIT_SUCCESS;
}