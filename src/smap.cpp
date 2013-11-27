/*
 * smap2.cpp
 *
 *  Created on: Sep 23, 2013
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "STools.h"

#include <spl/SSLib.h>
#ifdef SSLIB_USE_CGAL

#include <fstream>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <boost/range/iterator_range.hpp>

#include <spl/analysis/AnchorArrangement.h>
#include <spl/analysis/ArrangementSmoothing.h>
#include <spl/analysis/GnuplotAnchorArrangementPlotter.h>
#include <spl/analysis/VectorAnchorArrangementOutputter.h>
#include <spl/analysis/VoronoiEdgeTracer.h>

// FORWARD DECLARES //////////////////////////
class DataRow;

// NAMESPACES ////////////////////////////////
namespace fs = ::boost::filesystem;
namespace spla = ::spl::analysis;

// TYPEDEFS ////////////////////////////////////
typedef ::std::string LabelType;
typedef ::spl::analysis::AnchorArrangement< LabelType> AnchorArrangement;
typedef AnchorArrangement::EdgeTracer EdgeTracer;
typedef EdgeTracer::Point Point;
typedef ::std::vector< ::std::pair< Point, LabelType> > Points;
typedef ::spl::UniquePtr< spla::AnchorArrangementOutputter< LabelType> >::Type ArrangementOutputterPtr;

// CONSTANTS ////////////////////////////////

// CLASSES //////////////////////////////////
struct InputOptions
{
  ::std::string inputFile;
  int numSteps;
  double forceTolerance;
  bool dontSplitSharedVertices;
  double kappa;
  double vertexForceStrength;
  double areaForceStrength;
  ::std::string outputter;
};

// CONSTANTS /////////////////////////////////

// FUNCTIONS ////////////////////////////////
int
processCommandLineArgs(InputOptions & in, const int argc, char * argv[]);

void
smoothArrangement(const InputOptions & in, AnchorArrangement & arrangement);

ArrangementOutputterPtr
generateOutputter(const InputOptions & in);

int
main(const int argc, char * argv[])
{
  typedef boost::tokenizer< boost::char_separator< char> > Tok;

  using ::boost::lexical_cast;
  using ::spl::analysis::AnchorPoint;
  using ::std::abs;
  using ::std::fabs;
  using ::std::pow;

  static const boost::char_separator< char> tokSep(" \t");

  feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);

// Program options
  InputOptions in;

  int result = processCommandLineArgs(in, argc, argv);
  if(result != 0)
    return result;

  if(!fs::exists(fs::path(in.inputFile)))
  {
    ::std::cerr << "Input file " << in.inputFile << " does not exist.\n";
    return 1;
  }

  Points points;
  ::std::ifstream inFile(in.inputFile.c_str());
  if(inFile.is_open())
  {
    ::std::string line;
    ::std::set< Point> pointSet;
    while(::std::getline(inFile, line))
    {
      if(!line.empty() && line[0] != '#')
      {
        Tok toker(line, tokSep);
        Tok::iterator it = toker.begin();
        if(it == toker.end())
          continue;

        Point::R::FT x, y;
        LabelType label;

        bool foundAll = false;
        try
        {
          x = lexical_cast< Point::R::FT>(*it);
          if(++it == toker.end())
            continue;

          y = lexical_cast< Point::R::FT>(*it);
          if(++it == toker.end())
            continue;

          label = lexical_cast< LabelType>(*it);
          foundAll = true;
        }
        catch(const ::boost::bad_lexical_cast & /*e*/)
        {
        }
        if(foundAll)
        {
          const Point pt(x, y);
          if(pointSet.insert(pt).second)
            points.push_back(::std::make_pair(pt, label));
          else
            ::std::cerr << "Point (" << x << ", " << y
                << ") found more than once, ignoring.\n";
        }
      }
    }

    inFile.close();
  }

  EdgeTracer::Delaunay tempDelaunay;
  tempDelaunay.insert(points.begin(), points.end());
  EdgeTracer::Voronoi voronoi(tempDelaunay, true);

  EdgeTracer tracer(voronoi, !in.dontSplitSharedVertices);

  AnchorArrangement arr(tracer);
  smoothArrangement(in, arr);

  ArrangementOutputterPtr outputter = generateOutputter(in);
  outputter->outputArrangement(arr);

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
        po::value< ::std::string>(&in.inputFile), "input file")("steps,n",
        po::value< int>(&in.numSteps)->default_value(
            spla::SmoothingOptions::MAX_STEPS_DEFAULT),
        "number of smoothing steps (-1 = until tolerance is satisfied)")(
        "force-tol,f",
        po::value< double>(&in.forceTolerance)->default_value(
            spla::SmoothingOptions::FORCE_TOL_DEFAULT),
        "force tolerance used as stopping criterion")("no-split-shared,s",
        po::value< bool>(&in.dontSplitSharedVertices)->default_value(false)->zero_tokens(),
        "do not split shared vertices")("kappa,k",
        po::value< double>(&in.kappa)->default_value(
            spla::SmoothingOptions::SMOOTHING_FORCE_DEFAULT),
        "torque spring force strength, used to smooth edges")("area-force,a",
        po::value< double>(&in.areaForceStrength)->default_value(
            spla::SmoothingOptions::AREA_FORCE_DEFAULT),
        "area force strength, used to keep maintain the area of plot regions")(
        "vertex-force,v",
        po::value< double>(&in.vertexForceStrength)->default_value(
            spla::SmoothingOptions::VERTEX_FORCE_DEFAULT),
        "vertex force strength, used to keep vertices joining 3 or more edges centered")(
        "outputter,O",
        po::value< ::std::string>(&in.outputter)->default_value("gnuplot"),
        "The method of outputting the final map.  Possible options: gnuplot, vector");

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

void
smoothArrangement(const InputOptions & in, AnchorArrangement & arrangement)
{
  ::spl::analysis::SmoothingOptions smoothingOptions;
  smoothingOptions.smoothingForceStrength = in.kappa;
  smoothingOptions.areaForceStrength = in.areaForceStrength;
  smoothingOptions.vertexForceStrength = in.vertexForceStrength;
  smoothingOptions.maxSteps = in.numSteps;
  smoothingOptions.forceTol = in.forceTolerance;
  ::spl::analysis::smoothArrangement(smoothingOptions, &arrangement);
}

ArrangementOutputterPtr
generateOutputter(const InputOptions & in)
{
  ArrangementOutputterPtr outputter;

  if(in.outputter == "vector")
    outputter.reset(new spla::VectorAnchorArrangementOutputter< LabelType>());
  else if(in.outputter == "gnuplot")
  {
    outputter.reset(new spla::GnuplotAnchorArrangementPlotter<LabelType>("map"));
  }
  else
    ::std::cerr << "Error: unrecognised outputter - " << in.outputter
        << ::std::endl;

  return outputter;
}

#endif /* SSLIB_USE_CGAL */
