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

#include <spl/analysis/Histogram.h>
#include <spl/common/AtomSpeciesDatabase.h>
#include <spl/common/Structure.h>
#include <spl/common/UnitCell.h>
#include <spl/io/StructureReadWriteManager.h>

// From StructurePipe
#include <utility/PipeDataInitialisation.h>

// stools_common includes
#include <utility/TerminalFunctions.h>


// NAMESPACES ////////////////////////////////
namespace fs = ::boost::filesystem;
namespace ssa = ::spl::analysis;
namespace ssc = ::spl::common;
namespace ssio = ::spl::io;
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
  size_t maxDistancesPerPair;
  double distanceCutoff;
};

// CONSTANTS /////////////////////////////////
static const int RESULT_SUCCESS = 0;
static const int RESULT_GENERAL_FAILURE = 1;

// FUNCTION DECLARATIONS ///////////////////
Result processInputOptions(InputOptions & in, const int argc, char * argv[]);
Result calcLengths(const StructuresContainer & structures, const InputOptions & in);
void doLengths(const ssc::Structure & structure, const AtomPairs & pairs, const InputOptions & in);
void doHistogram(const ssc::Structure & structure, const AtomPairs & pairs, const InputOptions & in);
Result calcAngles(const StructuresContainer & structures, const InputOptions & in);
double calcCutoff(const ssc::Structure & structure, const InputOptions & in);

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
    rwMan.readStructures(structures, loc);
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
      //("angles,a", po::value<bool>(&in.calcAngles)->default_value(false)->zero_tokens(), "calculate angles")
      //("atoms-numbers,n", po::value< ::std::vector<unsigned int> >(&in.atomNumbers), "atom numbers")
      ("histogram,h", po::value<bool>(&in.histogramMode)->default_value(false)->zero_tokens(), "histogram mode")
      ("bin-width,w", po::value<double>(&in.histogramBinWidth)->default_value(0.0), "bin width (0.0 = automatically calculate)")
      ("graph,g", po::value<bool>(&in.histogramShowGraph)->default_value(false)->zero_tokens(), "show histogram ASCII graph")
      ("max-distances,m", po::value<size_t>(&in.maxDistancesPerPair)->default_value(0), "maximum number of distances per atom pair")
      ("cutoff,c", po::value<double>(&in.distanceCutoff)->default_value(0.0), "distances cutoff")
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
  if(stools::utility::isStdInPipedOrFile())
  {
    while(std::cin >> lineInput)
      in.inputFiles.push_back(lineInput);
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
    for(int i = 0; i < static_cast<int>(structure.getNumAtoms()) - 1; ++i)
      for(int j = i; j < static_cast<int>(structure.getNumAtoms()); ++j)
        pairs.push_back(AtomPair(i, j));

    const ssio::ResourceLocator * const locator = structure.getProperty(ssc::structure_properties::io::LAST_ABS_FILE_PATH);
    ::std::cout << locator->path() << "\n";
    if(in.histogramMode)
      doHistogram(structure, pairs, in);
    else
      doLengths(structure, pairs, in);

    pairs.clear();
  }

  return RESULT_SUCCESS;
}

void doLengths(const ssc::Structure & structure, const AtomPairs & pairs, const InputOptions & in)
{
  const double cutoff = calcCutoff(structure, in);
  const ssc::DistanceCalculator & distCalc = structure.getDistanceCalculator();

  size_t startOffset, numDists;
  BOOST_FOREACH(const AtomPair & pair, pairs)
  {
    ::std::vector<double> dists;
    distCalc.getDistsBetween(structure.getAtom(pair.first), structure.getAtom(pair.second), cutoff, dists);
    ::std::sort(dists.begin(), dists.end());

    // Skip the first entry for same atoms as the shortest distance will always be 0
    startOffset = !dists.empty() && pair.first == pair.second ? 1 : 0;

    if(in.maxDistancesPerPair == 0)
      numDists = dists.size() - startOffset;
    else
      numDists = ::std::min(in.maxDistancesPerPair, dists.size() - startOffset);

    ::std::stringstream ss;
    ss << pair.first << "(" << structure.getAtom(pair.first).getSpecies() << ")" <<
        "-" << pair.second << "(" << structure.getAtom(pair.second).getSpecies() << ")" << ": ";
    for(size_t i = startOffset; i < startOffset + numDists; ++i)
      ss << dists[i] << " ";
    ::std::cout << ss.str() << "\n";
  }
}

void doHistogram(const ssc::Structure & structure, const AtomPairs & pairs, const InputOptions & in)
{
  const double cutoff = calcCutoff(structure, in);
  const ssc::DistanceCalculator & distCalc = structure.getDistanceCalculator();
  ::std::vector<double> allDists;
  size_t startOffset, numDists;
  BOOST_FOREACH(const AtomPair & pair, pairs)
  {
    ::std::vector<double> dists;
    distCalc.getDistsBetween(structure.getAtom(pair.first), structure.getAtom(pair.second), cutoff, dists);
    ::std::sort(dists.begin(), dists.end());

    // Skip the first entry for same atoms as the shortest distance will always be 0
    startOffset = !dists.empty() && pair.first == pair.second ? 1 : 0;

    if(in.maxDistancesPerPair == 0)
      numDists = dists.size() - startOffset;
    else
      numDists = ::std::min(in.maxDistancesPerPair, dists.size() - startOffset);

    allDists.insert(allDists.end(), dists.begin() + startOffset, dists.begin() + startOffset + numDists);
  }

  ssa::Histogram hist(ssa::Histogram::estimateBinWidth(allDists.begin(), allDists.end(), 2.0, 80));
  hist.insert(allDists.begin(), allDists.end());

  // Print the histogram values
  for(size_t i = 0; i < hist.numBins(); ++i)
    ::std::cout << i << ": " << hist.getFrequency(i) << "\n";

  if(in.histogramShowGraph)
    ::std::cout << hist << "\n";
}

Result calcAngles(const StructuresContainer & structures, const InputOptions & in)
{
  SSLIB_DIE_NOT_IMPLEMENTED();
  return RESULT_GENERAL_FAILURE;
}

double calcCutoff(const ssc::Structure & structure, const InputOptions & in)
{
  if(in.distanceCutoff != 0.0)
    return in.distanceCutoff;
  else if(structure.getUnitCell())
    return 2.0 * ::std::pow(structure.getUnitCell()->getVolume(), 1.0 / 3.0);
  else // Assume cluster so give maximum distances cutoff
    return ::std::numeric_limits<double>::max();
}
