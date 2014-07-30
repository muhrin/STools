/*
 * shull.cpp
 *
 *  Created on: Jul 9, 2013
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

#include <spl/OptionalTypes.h>
#include <spl/analysis/ConvexHull.h>
#include <spl/analysis/GnuplotConvexHullPlotter.h>
#include <spl/analysis/StructureConvexHullInfoSupplier.h>
#include <spl/common/Structure.h>
#include <spl/common/Types.h>
#include <spl/io/ResourceLocator.h>
#include <spl/io/StructureReadWriteManager.h>


// NAMESPACES ////////////////////////////////
namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace ssc = spl::common;
namespace ssio = spl::io;
namespace ssa = spl::analysis;

struct InputOptions
{
  std::vector< std::string> inputFiles;
  std::string outputter;
  bool flattenConvexProperty;
  bool hideTieLines;
  bool hideLabels;
  bool hideOffHull;
  std::vector< std::string> customEndpoints;
  std::string distanceStructure;
};

spl::UniquePtr< ssa::ConvexHullOutputter>::Type
generateOutputter(const InputOptions & in);

int
main(const int argc, char * argv[])
{
  const std::string exeName(argv[0]);

  // Input options
  InputOptions in;

  try
  {
    po::options_description desc(
        "Usage: " + exeName + " [options] files...\nOptions:");
    desc.add_options()("help", "Show help message")("input-file",
        po::value< std::vector< std::string> >(&in.inputFiles), "input file(s)")(
        "outputter,o",
        po::value< std::string>(&in.outputter)->default_value("gnuplot"),
        "the hull outputter to use: gnuplot")("flatten,f",
        po::value< bool>(&in.flattenConvexProperty)->default_value(false)->zero_tokens(),
        "don't output convex property dimensions, only points that lie on the hull.")(
        "hide-tie-lines,t",
        po::value< bool>(&in.hideTieLines)->default_value(false)->zero_tokens(),
        "don't draw tie lines (visual outputters only)")("hide-labels,l",
        po::value< bool>(&in.hideLabels)->default_value(false)->zero_tokens(),
        "don't output labels for hull points")("hide-off-hull,h",
        po::value< bool>(&in.hideOffHull)->default_value(false)->zero_tokens(),
        "hide points not on the hull")("endpoints,e",
        po::value< std::vector< std::string> >(&in.customEndpoints)->multitoken(),
        "list of whitespace separated endpoints e.g. SiNi Fe")("distance,d",
        po::value< std::string>(&in.distanceStructure),
        "calculate distance from hull to given structure");

    po::positional_options_description p;
    p.add("input-file", -1);

    po::variables_map vm;
    po::store(
        po::command_line_parser(argc, argv).options(desc).positional(p).run(),
        vm);

    // Deal with help first, otherwise missing required parameters will cause exception
    if(vm.count("help"))
    {
      std::cout << desc << std::endl;
      return 1;
    }

    po::notify(vm);
  }
  catch(std::exception& e)
  {
    std::cout << e.what() << std::endl;
    return 1;
  }

  std::vector< ssc::AtomsFormula> endpoints(in.customEndpoints.size());
  if(!in.customEndpoints.empty())
  {
    bool error = false;
    for(size_t i = 0; i < in.customEndpoints.size(); ++i)
    {
      if(!endpoints[i].fromString(in.customEndpoints[i]))
      {
        std::cerr << "Unrecognised endpoint: " << in.customEndpoints[i]
            << std::endl;
        error = true;
      }
    }
    if(error)
      return 1;
  }

  ssio::StructureReadWriteManager rwMan;

  ssio::StructuresContainer loadedStructures;
  ssio::ResourceLocator structureLocator;

  BOOST_FOREACH(const std::string & inputFile, in.inputFiles)
  {
    if(!structureLocator.set(inputFile))
    {
      std::cerr << "Invalid structure path " << inputFile << ". Skipping."
          << std::endl;
      continue;
    }
    if(!fs::exists(structureLocator.path()))
    {
      std::cerr << "File " << inputFile << " does not exist.  Skipping."
          << std::endl;
      continue;
    }

    rwMan.readStructures(loadedStructures, structureLocator);
  }

  if(endpoints.empty())
    endpoints = ssa::ConvexHull::generateEndpoints(loadedStructures.begin(),
        loadedStructures.end());

  if(endpoints.size() < 2)
  {
    std::cerr
        << "Must have 2 or more endpoints to construct hull, currently have: ";
    BOOST_FOREACH(const ssc::AtomsFormula & formula, endpoints)
      std::cerr << formula << " ";
    std::cerr << std::endl;
    return 1;
  }

  ssa::ConvexHull hullGenerator(endpoints);
  const std::vector< ssa::ConvexHull::PointId> structureIds =
      hullGenerator.addStructures(loadedStructures.begin(),
          loadedStructures.end());

  // Try to get the distance structure (if any)
  ssc::StructurePtr distanceStructure;
  if(!in.distanceStructure.empty())
  {
    if(!structureLocator.set(in.distanceStructure))
    {
      std::cerr << "Invalid distance structure path " << in.distanceStructure
          << std::endl;
      return 1;
    }
    if(!fs::exists(structureLocator.path()))
    {
      std::cerr << "File " << in.distanceStructure << " does not exist"
          << std::endl;
      return 1;
    }
    distanceStructure = rwMan.readStructure(structureLocator);
    if(!distanceStructure.get())
    {
      std::cerr << "Error reading distance structure." << std::endl;
      return 1;
    }
  }

  if(hullGenerator.getHull())
  {
    if(distanceStructure.get())
    {
      spl::OptionalDouble dist = hullGenerator.distanceToHull(
          *distanceStructure);
      if(dist)
        std::cout << *dist << std::endl;
      else
        std::cerr << "Failed to calculate distance to hull" << std::endl;
    }
    else
    {
      // Create the info supplier
      ssa::StructureConvexHullInfoSupplier infoSupplier;
      for(size_t i = 0; i < structureIds.size(); ++i)
        infoSupplier.addStructure(loadedStructures[i], structureIds[i]);

      spl::UniquePtr< ssa::ConvexHullOutputter>::Type outputter =
          generateOutputter(in);
      if(outputter.get())
        outputter->outputHull(hullGenerator, infoSupplier);
    }
  }

  return 0;
}

spl::UniquePtr< ssa::ConvexHullOutputter>::Type
generateOutputter(const InputOptions & in)
{
  spl::UniquePtr< ssa::ConvexHullOutputter>::Type outputter;

  if(in.outputter == "gnuplot")
  {
    ssa::GnuplotConvexHullPlotter * gnuplot =
        new ssa::GnuplotConvexHullPlotter();
    gnuplot->setSupressEnergyDimension(in.flattenConvexProperty);
    gnuplot->setDrawTieLines(!in.hideTieLines);
    gnuplot->setDrawHullLabels(!in.hideLabels);
    gnuplot->setDrawOffHullPoints(!in.hideOffHull);
    outputter.reset(gnuplot);
  }
  else
    std::cerr << "Error: unrecognised outputter - " << in.outputter
        << std::endl;

  return outputter;
}
