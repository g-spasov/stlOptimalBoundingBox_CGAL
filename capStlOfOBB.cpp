#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/optimal_bounding_box.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Real_timer.h>
#include <CGAL/Vector_3.h>
#include <CGAL/IO/STL.h>
#include <CGAL/boost/graph/generators.h>
#include <CGAL/boost/graph/IO/STL.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <filesystem>
#include <CGAL/Aff_transformation_3.h>
namespace PMP = CGAL::Polygon_mesh_processing;
typedef CGAL::Exact_predicates_inexact_constructions_kernel    K;
typedef K::Point_3                                             Point;
typedef K::Vector_3                                            Vector;
typedef CGAL::Surface_mesh<Point>                              Surface_mesh;

namespace fs = std::filesystem;


int main(int argc, char** argv)
{

  if(argc <2){
    std::cout << "Usage: " << argv[0] << " [imput mesh file]" << std::endl;
    return EXIT_FAILURE;
  }
  const std::string filename =argv[1]; // (argc > 1) ? argv[1] : CGAL::data_file_path("meshes/pig.stl");
  const std::string saveFile = filename.substr(0,filename.find("."));

  Surface_mesh sm;
  if(!PMP::IO::read_polygon_mesh(filename, sm) || sm.is_empty())
  {
    std::cerr << "Invalid input file." << std::endl;
    return EXIT_FAILURE;
  }
  CGAL::Real_timer timer;

  std::cout << "Done reading the mesh" << std::endl;
  timer.start();
  // Compute the extreme points of the mesh, and then a tightly fitted oriented bounding box
  std::array<Point, 8> obb_points;
  CGAL::oriented_bounding_box(sm, obb_points,
                              CGAL::parameters::use_convex_hull(true));
  std::cout << "Elapsed time: " << timer.time() << std::endl;
  // Make a mesh out of the oriented bounding box
  Surface_mesh cap1,cap2,cap3,cap4,cap5,cap6;

  CGAL::Scaling S;
  CGAL::Aff_transformation_3<K> T(S,1e-3,1.0);
  //S(1e-3,1.0);
  //T(	1e-3,0,0,0, 0,1e-3,0,0,0,0,1e-3,0,1);

for(int i=0;i<8;i++){
  obb_points[i]=obb_points[i].transform(T);
}

  CGAL::make_quad(obb_points[0], obb_points[1], obb_points[2], obb_points[3], cap1);
  CGAL::make_quad(obb_points[4], obb_points[5], obb_points[6], obb_points[7], cap2);
  CGAL::make_quad(obb_points[0], obb_points[1], obb_points[6], obb_points[5], cap3);
  CGAL::make_quad(obb_points[1], obb_points[2], obb_points[7], obb_points[6], cap4);
  CGAL::make_quad(obb_points[0], obb_points[3], obb_points[4], obb_points[5], cap5);
  CGAL::make_quad(obb_points[2], obb_points[3], obb_points[4], obb_points[7], cap6);
  
  //Triangulate new meshes
  PMP::triangulate_faces(cap1);
  PMP::triangulate_faces(cap2);
  PMP::triangulate_faces(cap3);
  PMP::triangulate_faces(cap4);
  PMP::triangulate_faces(cap5);
  PMP::triangulate_faces(cap6);

  //fs::current_path(fs::temp_directory_path());
  std::cout << "Creating directory and saving box-caps stls" << std::endl;
  fs::create_directories("./box_cap/"+saveFile+"/");

  std::ofstream out("./box_cap/"+saveFile+"/cap1.stl");
  CGAL::IO::write_STL(out, cap1);
  out.close();

  out.open("./box_cap/"+saveFile+"/cap2.stl");
  CGAL::IO::write_STL(out, cap2);
  out.close();

  out.open("./box_cap/"+saveFile+"/cap3.stl");
  CGAL::IO::write_STL(out, cap3);
  out.close();

  out.open("./box_cap/"+saveFile+"/cap4.stl");
  CGAL::IO::write_STL(out, cap4);
  out.close();

  out.open("./box_cap/"+saveFile+"/cap5.stl");
  CGAL::IO::write_STL(out, cap5);
  out.close();

  out.open("./box_cap/"+saveFile+"/cap6.stl");
  CGAL::IO::write_STL(out, cap6);
  out.close();

  std::cout << "Done saving box-caps stls" << std::endl;

  return EXIT_SUCCESS;
}