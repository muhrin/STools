/*
 * strian.cpp
 *
 *  Created on: Jul 26, 2013
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////

#include <iostream>
#include <list>
#include <set>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>

// TEMP
#include <CGAL/Cartesian.h>
#include <CGAL/Projection_traits_xy_3.h>

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_3.h>

#include <CGAL/IO/Geomview_stream.h>
#include <CGAL/IO/Triangulation_geomview_ostream_2.h>
#include <CGAL/IO/Triangulation_geomview_ostream_3.h>

#include <CGAL/intersections.h>

// From Pipelib //


#include <spl/analysis/StructureTriangulation.h>
#include <spl/common/Structure.h>
#include <spl/common/Types.h>
#include <spl/io/ResourceLocator.h>
#include <spl/io/StructureReadWriteManager.h>

// From StructurePipe
#include <utility/PipeDataInitialisation.h>

// My includes //

// NAMESPACES ////////////////////////////////
namespace fs = ::boost::filesystem;
namespace po = ::boost::program_options;
namespace sp = ::spipe;
namespace ssc = ::spl::common;
namespace ssio = ::spl::io;
namespace ssa = ::spl::analysis;

struct InputOptions
{
  ::std::string inputFile;
  ::std::vector<int> atomIndices;
};


int main(const int argc, char * argv[])
{

  const ::std::string exeName(argv[0]);

  // Input options
  InputOptions in;

  try
  {
    po::options_description desc("Usage: " + exeName + " [options] files...\nOptions:");
    desc.add_options()
      ("help", "Show help message")
      ("input-file,i", po::value< ::std::string>(&in.inputFile), "input structure file")
      ("atom-indices,a", po::value< ::std::vector<int> >(&in.atomIndices)->multitoken(), "atom indices")
    ;

    po::positional_options_description p;
    p.add("input-file", -1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);

    // Deal with help first, otherwise missing required parameters will cause exception
    if(vm.count("help"))
    {
      ::std::cout << desc << ::std::endl;
      return 1;
    }

    po::notify(vm);
  }
  catch(std::exception& e)
  {
    ::std::cout << e.what() << ::std::endl;
    return 1;
  }


  ssio::StructureReadWriteManager rwMan;
  sp::utility::initStructureRwManDefault(rwMan);

  ssio::ResourceLocator structureLocator;
  if(!structureLocator.set(in.inputFile))
    ::std::cerr << "Invalid structure path " << in.inputFile << ". Skipping." << ::std::endl;
  if(!fs::exists(structureLocator.path()))
    ::std::cerr << "File " << in.inputFile << " does not exist.  Skipping." << ::std::endl;

  ssc::StructurePtr structure = rwMan.readStructure(structureLocator);
  if(!structure.get())
  {
    ::std::cerr << "Failed to load structure " << structureLocator << ::std::endl;
    return 1;
  }

  ssa::StructureTriangulation delaunay(*structure);

  BOOST_FOREACH(const int index, in.atomIndices)
  {
    ::std::cout << index << ": " << delaunay.getCoordination(structure->getAtom(index)) << ::std::endl;
  }

  return 0;
}



