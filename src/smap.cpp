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

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <boost/range/iterator_range.hpp>

#include <spl/analysis/ArrangementMapOutputter.h>
#include <spl/analysis/MapArrangementTraits.h>
#include <spl/analysis/MatplotlibMapOutputter.h>
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
typedef spl::UniquePtr< spla::ArrangementMapOutputter< MapTraits> >::Type MapOutputterPtr;
typedef MapTraits::Arrangement Map;
typedef std::map< LabelType, spla::ArrangementMapOutputter< MapTraits>::LabelProperties  > LabelsInfo;

// CONSTANTS ////////////////////////////////

// CLASSES //////////////////////////////////
struct InputOptions
{
  std::string inputFile;
  bool dontSplitSharedVertices;
  std::string outputter;
  std::string labelsInfoFile;
};

struct TableRow : public spla::ArrangementMapOutputter< MapTraits>::LabelProperties
{
  static TableRow
  fromStrings(const std::vector< std::string> & strings)
  {
    SSLIB_ASSERT(strings.size() == 2);

    TableRow info;
    if(!strings[0].empty())
      info.name = strings[0];

    if(!strings[1].empty())
    {
      try
      {
        std::stringstream ss(strings[1]);
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
    std::vector< std::string> strings(2);
    if(name)
      strings[0] = *name;
    if(colour)
      strings[1] = boost::lexical_cast< std::string>(*colour);
    return strings;
  }
};

// CONSTANTS /////////////////////////////////

// FUNCTIONS ////////////////////////////////
int
processCommandLineArgs(InputOptions & in, const int argc, char * argv[]);

MapOutputterPtr
generateOutputter(const InputOptions & in);

LabelsInfo
loadLabelsInfo(const InputOptions & in);

int
main(const int argc, char * argv[])
{
  typedef boost::tokenizer< boost::char_separator< char> > Tok;

  using boost::lexical_cast;

  static const boost::char_separator< char> tokSep(" \t");

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
          x = lexical_cast< Point::R::FT>(*it);
          if(++it == toker.end())
          continue;

          y = lexical_cast< Point::R::FT>(*it);
          if(++it == toker.end())
          continue;

          label = lexical_cast< LabelType>(*it);
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

  const LabelsInfo & labelsInfo = loadLabelsInfo(in);

  MapOutputterPtr outputter = generateOutputter(in);
  const std::string filename = spl::io::stemString(in.inputFile) + "_map."
      + outputter->fileExtension();
  std::ofstream outFile(filename.c_str());
  if(outFile.is_open())
  {
    outputter->outputArrangement(arr, labelsInfo, &outFile);
    outFile.close();
  }
  else
    std::cerr << "ERROR: Failed to open " << filename << "\n";

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
        po::value< std::string>(&in.inputFile), "input file")("steps,n",
        po::value< std::string>(&in.outputter)->default_value("matplotlib"),
        "The method of outputting the final map.  Possible options: matplotlib")
        ("labels_info,l", po::value< std::string>(&in.labelsInfoFile),
            "The file containing information about the labels. \
            The format should be:\n\t[label], [name], [colour integer]\n. \
            If not specified smap will look for a file called [input].labels")
        ;

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

  if(in.outputter == "matplotlib")
    out.reset(new spla::MatplotlibMapOutputter< MapTraits>());
  else
    std::cerr << "Error: unrecognised outputter - " << in.outputter
      << std::endl;

  return out;
}

LabelsInfo
loadLabelsInfo(const InputOptions & in)
{
  typedef spl::utility::TypedAssociativeTable< LabelType, TableRow > LabelsTable;

  // If the user hasn't supplied a filename then try the input file + .labels
  const std::string infoFilename = in.labelsInfoFile.empty() ?
      spl::io::stemString(in.inputFile) + ".labels" : in.labelsInfoFile;

  LabelsInfo info;

  if(fs::exists(infoFilename))
  {
    const LabelsTable & table = LabelsTable::load(infoFilename, ",");

    BOOST_FOREACH(LabelsTable::const_reference entry, table)
      info[entry.first] = entry.second;
  }

  return info;
}

#endif /* SPL_USE_CGAL */
