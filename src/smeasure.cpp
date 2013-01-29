/*
 * sinfo.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////

#include <sstream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>

// From SSLib //
#include <analysis/Histogram.h>
#include <common/AtomSpeciesDatabase.h>
#include <common/Structure.h>
#include <io/StructureReadWriteManager.h>

// From StructurePipe
#include <utility/PipeDataInitialisation.h>

// stools_common includes
#include <utility/TerminalFunctions.h>

// My includes //


// NAMESPACES ////////////////////////////////
namespace fs = ::boost::filesystem;
namespace ssa = ::sstbx::analysis;
namespace ssc = ::sstbx::common;
namespace ssio = ::sstbx::io;
namespace sp = ::spipe;

// TYPEDEFS //////////////////////////////////
typedef ssio::StructuresContainer StructuresContainer;
typedef int Result;
typedef ::std::pair<unsigned int, unsigned int> AtomPair;
typedef ::std::vector<AtomPair> AtomPairs;

// CLASSES ///////////////////////////////////
struct InputOptions
{
  ::std::vector< ::std::string> inputFiles;
  ::std::vector<unsigned int> atomNumbers;
  bool calcLengths;
  bool calcAngles;
  bool histogramMode;
  bool histogramShowGraph;
  double histogramBinWidth;
};

// CONSTANTS /////////////////////////////////
static const int RESULT_SUCCESS = 0;
static const int RESULT_GENERAL_FAILURE = 1;

// FUNCTION DECLARATIONS ///////////////////
Result processInputOptions(InputOptions & in, const int argc, char * argv[]);
Result calcLengths(const StructuresContainer & structures, const InputOptions & in);
void doLengths(const ssc::Structure & structure, const AtomPairs & pairs);
double calculateBinWidth(const ssc::Structure & structure, const AtomPairs & pairs);
void doHistogram(const ssc::Structure & structure, const AtomPairs & pairs, const InputOptions & in);
Result calcAngles(const StructuresContainer & structures, const InputOptions & in);

int main(const int argc, char * argv[])
{

  ssio::StructureReadWriteManager rwMan;
  sp::utility::initStructureRwManDefault(rwMan);

  // Process input and detect errors
  InputOptions in;
  Result result = processInputOptions(in, argc, argv);
  if(result != RESULT_SUCCESS)
    return result;

  ssc::AtomSpeciesDatabase speciesDb;
  StructuresContainer structures;
  ssio::ResourceLocator loc;
  BOOST_FOREACH(const ::std::string & inputFile, in.inputFiles)
  {
    loc.set(inputFile);
    rwMan.readStructures(structures, loc, speciesDb);
  }

  if(in.calcLengths)
    calcLengths(structures, in);
  if(in.calcAngles)
    calcAngles(structures, in);

  return 0;
}

Result processInputOptions(InputOptions & in, const int argc, char * argv[])
{
  namespace po = ::boost::program_options;

  const ::std::string exeName(argv[0]);
  ::std::vector< ::std::string> DEFAULT_INPUT_FILES(1, fs::current_path().string());

  try
  {
    po::options_description desc("Usage: " + exeName + " [options] files...\n" +
      "\nOptions");
    desc.add_options()
      ("help", "Show help message")
      ("input,i", po::value< ::std::vector< ::std::string> >(&in.inputFiles), "input structure file(s)")
      ("lengths,l", po::value<bool>(&in.calcLengths)->default_value(true)->zero_tokens(), "calculate lengths")
      ("angles,a", po::value<bool>(&in.calcAngles)->default_value(false)->zero_tokens(), "calculate angles")
      ("atoms-numbers,n", po::value< ::std::vector<unsigned int> >(&in.atomNumbers), "atom numbers")
      ("histogram,h", po::value<bool>(&in.histogramMode)->default_value(false)->zero_tokens(), "histogram mode")
      ("bin-width,w", po::value<double>(&in.histogramBinWidth)->default_value(0.0), "bin width (0.0 = automatically calculate)")
      ("graph,g", po::value<bool>(&in.histogramShowGraph)->default_value(false)->zero_tokens(), "show histogram ASCII graph")
    ;

    po::positional_options_description p;
    p.add("input", -1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);

    // Deal with help first, otherwise missing required parameters will cause exception
    if(vm.count("help"))
    {
      ::std::cout << desc << ::std::endl;
      return RESULT_GENERAL_FAILURE;
    }

    po::notify(vm);
  }
  catch(std::exception& e)
  {
    ::std::cerr << e.what() << ::std::endl;
    return RESULT_GENERAL_FAILURE;
  }

  // Get any input from standard in (piped)
  std::string lineInput;
  bool foundPipedInput = false;
  if(stools::utility::isStdInPipedOrFile())
  {
    while(std::cin >> lineInput)
    {
      in.inputFiles.push_back(lineInput);
      foundPipedInput = true;
    }
  }

  if(in.inputFiles.empty())
    in.inputFiles = DEFAULT_INPUT_FILES;

  return RESULT_SUCCESS;
}


Result calcLengths(const StructuresContainer & structures, const InputOptions & in)
{
  AtomPairs pairs;
  BOOST_FOREACH(const ssc::Structure & structure, structures)
  {
    for(unsigned int i = 0; i < structure.getNumAtoms() - 1; ++i)
      for(unsigned int j = i + 1; j < structure.getNumAtoms(); ++j)
        pairs.push_back(AtomPair(i, j));

    if(in.histogramMode)
      doHistogram(structure, pairs, in);
    else
      doLengths(structure, pairs);

    pairs.clear();
  }

  return RESULT_SUCCESS;
}

void doLengths(const ssc::Structure & structure, const AtomPairs & pairs)
{
  const ssc::DistanceCalculator & distCalc = structure.getDistanceCalculator();
  double dist;
  ::std::stringstream ss;
  BOOST_FOREACH(const AtomPair & pair, pairs)
  {
    dist = distCalc.getDistMinImg(structure.getAtom(pair.first), structure.getAtom(pair.second));
    ss.str(""); // Reset the stringstream
    ss << pair.first << "-" << pair.second << ": " << dist;
    ::std::cout << ss.str() << ::std::endl;
  }
}

double calculateBinWidth(const ssc::Structure & structure, const AtomPairs & pairs)
{
  const ssc::DistanceCalculator & distCalc = structure.getDistanceCalculator();

  // First calculate the maximum distance
  double maxDist = 0.0;
  BOOST_FOREACH(const AtomPair & pair, pairs)
  {
    maxDist = ::std::max(
      distCalc.getDistMinImg(structure.getAtom(pair.first), structure.getAtom(pair.second)),
      maxDist
    );
  }

  // Calculate the bin width
  return 30 * maxDist / static_cast<double>(pairs.size());

}

void doHistogram(const ssc::Structure & structure, const AtomPairs & pairs, const InputOptions & in)
{
  double binWidth = in.histogramBinWidth;
  if(binWidth == 0.0)
    binWidth = calculateBinWidth(structure, pairs);

  ssa::Histogram hist(binWidth);
  const ssc::DistanceCalculator & distCalc = structure.getDistanceCalculator();
  double dist;
  BOOST_FOREACH(const AtomPair & pair, pairs)
  {
    dist = distCalc.getDistMinImg(structure.getAtom(pair.first), structure.getAtom(pair.second));
    hist.insert(dist);
  }

  // Print the histogram values
  for(size_t i = 0; i < hist.numBins(); ++i)
    ::std::cout << i << ": " << hist.getValue(i) << ::std::endl;

  if(in.histogramShowGraph)
    ::std::cout << hist << ::std::endl;
}

Result calcAngles(const StructuresContainer & structures, const InputOptions & in)
{

  return RESULT_SUCCESS;
}
