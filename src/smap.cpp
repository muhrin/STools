/*
 * smap.cpp
 *
 *  Created on: Sep 23, 2013
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "STools.h"

#include <spl/SSLib.h>
#ifdef SPL_USE_CGAL

#include <fstream>
#include <string>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <boost/range/iterator_range.hpp>

#include <spl/analysis/ArrangementMapOutputter.h>
#include <spl/analysis/MapArrangementTraits.h>
#include <spl/analysis/MatplotlibMapOutputter.h>
#include <spl/analysis/RawMapOutputter.h>
#include <spl/analysis/VoronoiPathTracer.h>
#include <spl/io/BoostFilesystem.h>
#include <spl/utility/DataTable.h>

#include "utility/TerminalFunctions.h"

// FORWARD DECLARES //////////////////////////
class DataRow;

// NAMESPACES ////////////////////////////////
using namespace stools;

namespace fs = boost::filesystem;
namespace spla = spl::analysis;

// TYPEDEFS ////////////////////////////////////
typedef std::string LabelType;
typedef spl::analysis::MapArrangementTraits< LabelType> MapTraits;
typedef spl::analysis::VoronoiPathTracer< MapTraits> Tracer;
typedef Tracer::Point Point;
typedef std::vector< Tracer::PointLabel> Points;
typedef spla::ArrangementMapOutputter< MapTraits> Outputter;
typedef spl::UniquePtr< Outputter >::Type MapOutputterPtr;
typedef MapTraits::Arrangement Map;

// CONSTANTS ////////////////////////////////

// CLASSES //////////////////////////////////
struct InputOptions
{
  std::string inputFile;
  bool dontSplitSharedVertices;
  std::string outputter;
  std::string labelsInfoFile;
  std::string xLabel;
  std::string yLabel;
};

struct TableRow
{
  boost::optional<int> colour;

  static TableRow
  fromStrings(const std::vector< std::string> & strings)
  {
    SSLIB_ASSERT(strings.size() == 1);

    TableRow info;

    if(!strings[0].empty())
    {
      try
      {
        std::stringstream ss(strings[0]);
        int colour;
        ss >> std::hex >> colour;
        info.colour = colour;
      }
      catch(const std::exception & /*e*/)
      {
        utility::warning() << "Invalid colour value, " << strings[1]
        << ", in labels file.  Use format: 0xrrggbb.\n";
      }
    }

    return info;
  }

  std::vector< std::string>
  toStrings()
  {
    std::vector< std::string> strings(1);
    if(colour)
    strings[0] = boost::lexical_cast< std::string>(*colour);
    return strings;
  }
};

// CONSTANTS /////////////////////////////////

// FUNCTIONS ////////////////////////////////
int
processCommandLineArgs(InputOptions & in, const int argc, char * argv[]);

MapOutputterPtr
generateOutputter(const InputOptions & in);

void
applySettings(const InputOptions & in, Outputter * const outputter);

int
main(const int argc, char * argv[])
{
  typedef boost::tokenizer< boost::char_separator< char> > Tok;

  using boost::lexical_cast;
  using boost::algorithm::trim_copy;

  static const boost::char_separator< char> tokSep(",\t ");

  //feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);

  // Program options
  InputOptions in;

  int result = processCommandLineArgs(in, argc, argv);
  if(result != 0)
  return result;

  if(!fs::exists(fs::path(in.inputFile)))
  {
    std::cerr << "Input file " << in.inputFile << " does not exist.\n";
    return 1;
  }

  Points points;
  std::ifstream inFile(in.inputFile.c_str());
  if(inFile.is_open())
  {
    std::string line;
    std::set< Point> pointSet;
    while(std::getline(inFile, line))
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
          x = lexical_cast< Point::R::FT>(trim_copy(*it));
          if(++it == toker.end())
          continue;

          y = lexical_cast< Point::R::FT>(trim_copy(*it));
          if(++it == toker.end())
          continue;

          label = lexical_cast< LabelType>(trim_copy(*it));
          foundAll = true;
        }
        catch(const boost::bad_lexical_cast & /*e*/)
        {
        }
        if(foundAll)
        {
          const Point pt(x, y);
          if(pointSet.insert(pt).second)
          points.push_back(std::make_pair(pt, label));
          else
          std::cerr << "Point (" << x << ", " << y
          << ") found more than once, ignoring.\n";
        }
      }
    }

    inFile.close();
  }

  const Map arr = Tracer::processPath(points.begin(), points.end());

  MapOutputterPtr outputter = generateOutputter(in);
  applySettings(in, outputter.get());

  outputter->outputArrangement(arr);

  return 0;
}

int
processCommandLineArgs(InputOptions & in, const int argc, char * argv[])
{
  namespace po = boost::program_options;

  const std::string exeName(argv[0]);

  try
  {
    po::options_description general(
        "smap\nUsage: " + exeName + " [options] input_file...\nOptions");
    general.add_options()("help", "Show help message")("input-file",
        po::value< std::string>(&in.inputFile), "input file")("outputter,f",
        po::value< std::string>(&in.outputter)->default_value("raw"),
        "The method of outputting the final map.  Possible options: raw, matplotlib")
    ("labels_info,l", po::value< std::string>(&in.labelsInfoFile),
        "The file containing information about the labels. \
            The format should be:\n\t[label], [name], [colour integer]\n. \
            If not specified smap will look for a file called [input].labels")("xlabel,x",
        po::value< std::string>(&in.xLabel), "The x-axis label")("ylabel,y",
        po::value< std::string>(&in.yLabel), "The y-axis label");

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
      std::cout << cmdLineOptions << std::endl;
      return 1;
    }

    po::notify(vm);
  }
  catch(std::exception& e)
  {
    std::cout << e.what() << "\n";
    return 1;
  }

  // Everything went fine
  return 0;
}

MapOutputterPtr
generateOutputter(const InputOptions & in)
{
  MapOutputterPtr out;

  if(in.outputter == "raw")
    out.reset(new spla::RawMapOutputter<MapTraits>());
  else if(in.outputter == "matplotlib")
    out.reset(new spla::MatplotlibMapOutputter< MapTraits>());
  else
    std::cerr << "Error: unrecognised outputter - " << in.outputter << std::endl;

  return out;
}

void
applySettings(const InputOptions & in, Outputter * const outputter)
{
  typedef spl::utility::TypedAssociativeTable< LabelType, TableRow > LabelsTable;

  // If the user hasn't supplied a filename then try the input file + .labels
  const std::string infoFilename = in.labelsInfoFile.empty() ?
  spl::io::stemString(in.inputFile) + ".labels" : in.labelsInfoFile;

  if(fs::exists(infoFilename))
  {
    std::map< LabelType, int> colourMap;
    const LabelsTable & table = LabelsTable::load(infoFilename, ",");

    BOOST_FOREACH(LabelsTable::const_reference entry, table)
    {
      if(entry.second.colour)
        colourMap[entry.first] = *entry.second.colour;
    }

    outputter->setColourMap(colourMap);
  }

  outputter->setSeedName(spl::io::stemString(in.inputFile));
  if(!in.xLabel.empty())
    outputter->setXLabel(in.xLabel);
  if(!in.yLabel.empty())
    outputter->setYLabel(in.yLabel);

}

#endif /* SPL_USE_CGAL */
