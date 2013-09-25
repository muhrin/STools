/*
 * smap.cpp
 *
 *  Created on: Sep 23, 2013
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "STools.h"

#include <spl/SSLib.h>
#ifdef SSLIB_USE_CGAL

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/smart_ptr/scoped_array.hpp>
#include <boost/tokenizer.hpp>

// CGAL //
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_criteria_3.h>

#include <CGAL/Labeled_image_mesh_domain_3.h>
#include <CGAL/make_mesh_3.h>
#include <CGAL/Image_3.h>
#include <CGAL/ImageIO.h>

// FORWARD DECLARES //////////////////////////
class DataRow;

// MACROS ////////////////////////////////////
// Domain
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

typedef ::std::vector< DataRow> DataRows;

// NAMESPACES ////////////////////////////////
namespace fs = ::boost::filesystem;

// CLASSES //////////////////////////////////
struct InputOptions
{
  ::std::string inputFile;
};

struct DataRow
{
  double x, y, z;
  unsigned char label;
};

class DatasetProperties
{
public:
  DatasetProperties()
  {
    x0 = y0 = z0 = 0.0;
  }
  int
  xIndex(const double x) const
  {
    return ::std::floor((x - x0) / dx + 0.5);
  }
  int
  yIndex(const double y) const
  {
    return ::std::floor((y - y0) / dy + 0.5);
  }
  int
  zIndex(const double z) const
  {
    return ::std::floor((z - z0) / dz + 0.5);
  }
  int numX;
  int numY;
  int numZ;
  double x0, y0, z0;
  double dx, dy, dz;
};

class LabelledMap
{
public:
  typedef char Type;
  typedef K::RT RT;

  LabelledMap(const DataRows & rows, const DatasetProperties & properties);

  int
  xdim() const;
  int
  ydim() const;
  int
  zdim() const;

  K::RT
  vx() const;
  K::RT
  vy() const;
  K::RT
  vz() const;

  const Type *
  data() const;

private:
  int
  idx(const int x, const int y, const int z) const;

  int myNumX;
  int myNumY;
  int myNumZ;

  const double dX;
  const double dY;
  const double dZ;

  ::boost::scoped_array< Type> myLabels;
};

// TYPEDEFS /////////////////////////////////
typedef CGAL::Labeled_image_mesh_domain_3< CGAL::Image_3, K> Mesh_domain;

// Triangulation
typedef CGAL::Mesh_triangulation_3< Mesh_domain>::type Tr;
typedef CGAL::Mesh_complex_3_in_triangulation_3< Tr> C3t3;

// Criteria
typedef CGAL::Mesh_criteria_3< Tr> Mesh_criteria;
typedef CGAL::Mesh_constant_domain_field_3< Mesh_domain::R, Mesh_domain::Index> Sizing_field;

// CONSTANTS /////////////////////////////////

// FUNCTIONS ////////////////////////////////
int
processCommandLineArgs(InputOptions & in, const int argc, char * argv[]);

int
main(const int argc, char * argv[])
{
  typedef boost::tokenizer< boost::char_separator< char> > Tok;

  using namespace CGAL::parameters;
  using ::boost::lexical_cast;

  static const boost::char_separator< char> tokSep(" \t");

  // Program options
  InputOptions in;

  int result = processCommandLineArgs(in, argc, argv);
  if(result != 0)
    return result;

  if(!fs::exists(fs::path(in.inputFile)))
  {
    ::std::cerr << "Input file " << in.inputFile << " does not exist.";
    return 1;
  }

  DatasetProperties setInfo;

  setInfo.dx = 0.1;
  setInfo.dy = 0.1;
  setInfo.dz = 0.1;
  setInfo.x0 = 0.1;
  setInfo.y0 = 0.1;
  setInfo.z0 = 0.1;

  ::std::vector< DataRow> mapData;

  ::std::ifstream inFile(in.inputFile.c_str());
  if(inFile.is_open())
  {
    ::std::string line;
    while(::std::getline(inFile, line))
    {
      if(!line.empty() && line[0] != '#')
      {
        DataRow row;
        Tok toker(line, tokSep);
        Tok::iterator it = toker.begin();

        bool foundAll = false;
        try
        {
          if(it != toker.end())
            row.x = lexical_cast< double>(*it);
          ++it;
          if(it != toker.end())
            row.y = ::boost::lexical_cast< double>(*it);
          ++it;
          if(it != toker.end())
            row.z = ::boost::lexical_cast< double>(*it);
          ++it;
          if(it != toker.end())
            row.label = ::boost::lexical_cast< unsigned char>(*it);
          foundAll = true;
        }
        catch(const ::boost::bad_lexical_cast & /*e*/)
        {
        }
        if(foundAll)
          mapData.push_back(row);
      }
    }

    inFile.close();
  }

  // Now create the map
  // Domain
  LabelledMap map(mapData, setInfo);

  FILE * outFile = fopen("out.dat", "wb");
  fwrite(map.data(), static_cast< size_t>(map.xdim() * map.ydim() * map.zdim()),
      sizeof(LabelledMap::Type), outFile);
  fclose(outFile);

  CGAL::Image_3 image;
  image.read_raw("out.dat", map.xdim(), map.ydim(), map.zdim(), setInfo.dx,
      setInfo.dy, setInfo.dz);
  //image.read("test");
  Mesh_domain domain(image);

  // Mesh criteria
  Mesh_criteria criteria(facet_angle = 30, facet_size = 6, facet_distance = 4,
      cell_radius_edge_ratio = 3, cell_size = 8);

  // Meshing
  C3t3 c3t3 = CGAL::make_mesh_3< C3t3>(domain, criteria);

  // Output
  std::ofstream medit_file("out.mesh");
  c3t3.output_to_medit(medit_file);

  return 0;
}

int
processCommandLineArgs(InputOptions & in, const int argc, char * argv[])
{
  namespace po = ::boost::program_options;

  const ::std::string exeName(argv[0]);

  try
  {
    po::options_description general(
        "smap\nUsage: " + exeName + " [options] input_file...\nOptions");
    general.add_options()("help", "Show help message")("input-file",
        po::value< ::std::string>(&in.inputFile), "input file");

    po::positional_options_description p;
    p.add("input-file", 1);

    po::options_description cmdLineOptions;
    cmdLineOptions.add(general);

    po::variables_map vm;
    po::store(
        po::command_line_parser(argc, argv).options(cmdLineOptions).positional(
            p).run(), vm);

    // Deal with help first, otherwise missing required parameters will cause exception on vm.notify
    if(vm.count("help"))
    {
      ::std::cout << cmdLineOptions << ::std::endl;
      return 1;
    }

    po::notify(vm);
  }
  catch(std::exception& e)
  {
    ::std::cout << e.what() << "\n";
    return 1;
  }

  // Everything went fine
  return 0;
}

LabelledMap::LabelledMap(const DataRows & rows,
    const DatasetProperties & properties) :
    dX(properties.dx), dY(properties.dy), dZ(properties.dz)
{
  myNumX = myNumY = myNumZ = 0;
  BOOST_FOREACH(const DataRow & row, rows)
  {
    myNumX = ::std::max(myNumX, properties.xIndex(row.x));
    myNumY = ::std::max(myNumY, properties.yIndex(row.y));
    myNumZ = ::std::max(myNumZ, properties.zIndex(row.z));
  }

  myLabels.reset(new Type[myNumX * myNumY * myNumZ]);
  ::std::fill(myLabels.get(), myLabels.get() + myNumX * myNumY * myNumZ, 0);
  BOOST_FOREACH(const DataRow & row, rows)
  {
    myLabels[idx(properties.xIndex(row.x), properties.yIndex(row.y),
        properties.zIndex(row.z))] = static_cast< Type>(row.label);
  }
}

int
LabelledMap::xdim() const
{
  return myNumX;
}

int
LabelledMap::ydim() const
{
  return myNumY;
}

int
LabelledMap::zdim() const
{
  return myNumZ;
}

K::RT
LabelledMap::vx() const
{
  return RT(dX);
}

K::RT
LabelledMap::vy() const
{
  return RT(dY);
}

K::RT
LabelledMap::vz() const
{
  return RT(dZ);
}

const LabelledMap::Type *
LabelledMap::data() const
{
  return myLabels.get();
}

int
LabelledMap::idx(const int x, const int y, const int z) const
{
  return x * myNumX + y * myNumY + z;
}

#endif /* SSLIB_USE_CGAL */
