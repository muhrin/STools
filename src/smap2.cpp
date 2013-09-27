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
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>

// includes for defining the Voronoi diagram adaptor
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>

// FORWARD DECLARES //////////////////////////
class DataRow;

// TYPEDEFS ////////////////////////////////////
// typedefs for defining the adaptor
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2< K> DT;
typedef CGAL::Delaunay_triangulation_adaptation_traits_2< DT> AT;
typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2< DT> AP;
typedef CGAL::Voronoi_diagram_2< DT, AT, AP> VD;

typedef AT::Site_2 Site_2;

// NAMESPACES ////////////////////////////////
namespace fs = ::boost::filesystem;

// CLASSES //////////////////////////////////
struct InputOptions
{
  ::std::string inputFile;
};

struct PointData
{
  Site_2 site;
  int label;
};

// TYPEDEFS /////////////////////////////////
typedef ::std::vector< PointData> Points;

// CONSTANTS /////////////////////////////////

// FUNCTIONS ////////////////////////////////
int
processCommandLineArgs(InputOptions & in, const int argc, char * argv[]);

int
main(const int argc, char * argv[])
{
  typedef boost::tokenizer< boost::char_separator< char> > Tok;

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

  Points points;
  ::std::ifstream inFile(in.inputFile.c_str());
  if(inFile.is_open())
  {
    ::std::string line;
    PointData p;
    while(::std::getline(inFile, line))
    {
      if(!line.empty() && line[0] != '#')
      {
        Tok toker(line, tokSep);
        Tok::iterator it = toker.begin();

        double x, y;

        bool foundAll = false;
        try
        {
          if(it != toker.end())
            x = lexical_cast< double>(*it);
          ++it;
          if(it != toker.end())
            y = ::boost::lexical_cast< double>(*it);
          ++it;
          if(it != toker.end())
            p.label = ::boost::lexical_cast< int>(*it);
          foundAll = true;
        }
        catch(const ::boost::bad_lexical_cast & /*e*/)
        {
        }
        if(foundAll)
        {
          p.site = Site_2(x, y);
          points.push_back(p);
        }
      }
    }

    inFile.close();
  }

  VD voronoiDiagram;
  BOOST_FOREACH(const PointData & p, points)
  {
    voronoiDiagram.insert(p.site);
  }

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

#endif /* SSLIB_USE_CGAL */
