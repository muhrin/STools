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
  ::std::string outputter;
  bool flattenConvexProperty;
  bool hideTieLines;
  bool labelHullPoints;
  bool hideOffHull;
  ::std::vector< ::std::string> customEndpoints;
};

::sstbx::UniquePtr<ssa::IConvexHullOutputter>::Type generateOutputter(const InputOptions & in);

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
      ("outputter,o", po::value< ::std::string>(&in.outputter)->default_value("gnuplot"), "the hull outputter to use: gnuplot")
      ("flatten,f", po::value<bool>(&in.flattenConvexProperty)->default_value(false)->zero_tokens(), "don't output convex property dimensions, only points that lie on the hull.")
      ("hide-tie-lines,t", po::value<bool>(&in.hideTieLines)->default_value(false)->zero_tokens(), "don't draw tie lines (visual outputters only)")
      ("label,l", po::value<bool>(&in.labelHullPoints)->default_value(false)->zero_tokens(), "label hull points")
      ("hide-off-hull,h", po::value<bool>(&in.hideOffHull)->default_value(false)->zero_tokens(), "hide points not on the hull")
      ("endpoints,e", po::value< ::std::vector< ::std::string> >(&in.customEndpoints)->multitoken(), "list of whitespace separated endpoints e.g. SiNi Fe")
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

  ::std::vector<ssc::AtomsFormula> endpoints(in.customEndpoints.size());
  if(!in.customEndpoints.empty())
  {
    bool error = false;
    for(int i = 0; i < in.customEndpoints.size(); ++i)
    {
      if(!endpoints[i].fromString(in.customEndpoints[i]))
      {
        ::std::cerr << "Unrecognised endpoint: " << in.customEndpoints[i] << ::std::endl;
        error = true;
      }
    }
    if(error)
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

  if(endpoints.empty())
    endpoints = ssa::ConvexHull::generateEndpoints(loadedStructures.begin(), loadedStructures.end());

  ssa::ConvexHull hullGenerator(endpoints);
  const ::std::vector<ssa::ConvexHull::PointId> structureIds = hullGenerator.addStructures(loadedStructures.begin(), loadedStructures.end());


  ssa::StructureConvexHullInfoSupplier infoSupplier;
  for(int i = 0; i < structureIds.size(); ++i)
    infoSupplier.addStructure(loadedStructures[i], structureIds[i]);

  if(hullGenerator.getHull())
  {
    ::sstbx::UniquePtr<ssa::IConvexHullOutputter>::Type outputter = generateOutputter(in);
    if(outputter.get())
      outputter->outputHull(hullGenerator, &infoSupplier);
  }


  return 0;
}


::sstbx::UniquePtr<ssa::IConvexHullOutputter>::Type generateOutputter(const InputOptions & in)
{
  ::sstbx::UniquePtr<ssa::IConvexHullOutputter>::Type outputter;

  if(in.outputter == "gnuplot")
  {
    ssa::GnuplotConvexHullPlotter * gnuplot = new ssa::GnuplotConvexHullPlotter();
    gnuplot->setSupressEnergyDimension(in.flattenConvexProperty);
    gnuplot->setDrawTieLines(!in.hideTieLines);
    gnuplot->setDrawHullLabels(in.labelHullPoints);
    gnuplot->setDrawOffHullPoints(!in.hideOffHull);
    outputter.reset(gnuplot);
  }
  else
    ::std::cerr << "Error: unrecognised outputter - " << in.outputter << ::std::endl;

  return outputter;
}
