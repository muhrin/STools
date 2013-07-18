/*
 * shull.cpp
 *
 *  Created on: Jul 9, 2013
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////

#include <list>
#include <set>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>

// From Pipelib //


// From SSLib //
#include <analysis/ConvexHull.h>
#include <analysis/GnuplotConvexHullPlotter.h>
#include <analysis/StructureConvexHullInfoSupplier.h>
#include <common/Structure.h>
#include <common/Types.h>
#include <io/ResourceLocator.h>
#include <io/StructureReadWriteManager.h>

// From StructurePipe
#include <utility/PipeDataInitialisation.h>

// My includes //

// NAMESPACES ////////////////////////////////
namespace fs = ::boost::filesystem;
namespace po = ::boost::program_options;
namespace sp = ::spipe;
namespace ssc = ::sstbx::common;
namespace ssio = ::sstbx::io;
namespace ssa = ::sstbx::analysis;

struct InputOptions
{
  double precision;
  ::std::vector< ::std::string> inputFiles;
  bool printSgNumber;
};

int main(const int argc, char * argv[])
{
  typedef ssa::ConvexHull::Hull ConvexHull;

  const ::std::string exeName(argv[0]);

  // Input options
  InputOptions in;

  try
  {
    po::options_description desc("Usage: " + exeName + " [options] files...\nOptions:");
    desc.add_options()
      ("help", "Show help message")
      ("input-file", po::value< ::std::vector< ::std::string> >(&in.inputFiles), "input file(s)")
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

  ssio::StructuresContainer loadedStructures;
  ssio::ResourceLocator structureLocator;

  BOOST_FOREACH(const ::std::string & inputFile, in.inputFiles)
  {
    if(!structureLocator.set(inputFile))
    {
      ::std::cerr << "Invalid structure path " << inputFile << ". Skipping." << ::std::endl;
      continue;
    }
    if(!fs::exists(structureLocator.path()))
    {
      ::std::cerr << "File " << inputFile << " does not exist.  Skipping." << ::std::endl;
      continue;
    }

    rwMan.readStructures(loadedStructures, structureLocator);
  }

  ssa::ConvexHull hullGenerator(ssa::ConvexHull::generateEndpoints(loadedStructures.begin(), loadedStructures.end()));
  const ::std::vector<ssa::ConvexHull::PointId> structureIds = hullGenerator.addStructures(loadedStructures.begin(), loadedStructures.end());


  ssa::StructureConvexHullInfoSupplier infoSupplier;
  for(int i = 0; i < structureIds.size(); ++i)
    infoSupplier.addStructure(loadedStructures[i], structureIds[i]);

  if(hullGenerator.getHull())
  {
    ssa::GnuplotConvexHullPlotter plotter;
    plotter.outputHull(hullGenerator);//, infoSupplier);
//    ConvexHull::Hull_vertex_const_iterator it = hull->hull_vertices_begin();
//    const ConvexHull::Hull_vertex_const_iterator end = hull->hull_vertices_end();
//    for(; it != end; ++it)
//    {
//      ConvexHull::Point_d p = it->point();
//      ::std::cout << p.getId() << " ";
//      for(int i = 0; i < p.dimension(); ++i)
//        ::std::cout << CGAL::to_double(p[i]) << " ";
//      ::std::cout << ::std::endl;
//    }
  }


  return 0;
}
