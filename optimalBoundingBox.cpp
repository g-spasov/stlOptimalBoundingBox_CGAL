#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/optimal_bounding_box.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Real_timer.h>
#include <CGAL/Vector_3.h>
#include <CGAL/IO/STL.h>

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <filesystem>

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



  //Let's save the stl file of the bounding box

  fs::current_path(fs::temp_directory_path());
  fs::create_directories("boxes");

  std::ofstream out("boxes/"+saveFile+".stl");
  CGAL::IO::write_STL(out, obb_sm);
  out.close();

  Point center ={0,0,0};
  double_t x=0,y=0,z=0;

  for(int i=0;i<8;i++){
    x+=obb_points[i].x();
    y+=obb_points[i].y();
    z+=obb_points[i].z();
  }

  center = {x/8,y/8,z/8};
  Vector i(obb_points[0],obb_points[1]),j(obb_points[0],obb_points[3]),k(obb_points[0],obb_points[5]);

  i/=2;j/=2;k/=2;

  //Let us save in the saveFile the center and the vectors of the bounding box
  //The file has the following structure
  //name: [name of the file]
  //center: [x] [y] [z]
  //i: [x] [y] [z]
  //j: [x] [y] [z]
  //k: [x] [y] [z]
  std::ofstream save(saveFile+".txt");
  save << "name: " << saveFile << std::endl;
  save << "center: " << center << std::endl;
  save << "i: " << i << std::endl;
  save << "j: " << j << std::endl;
  save << "k: " << k << std::endl;
  save.close();


  return EXIT_SUCCESS;
}