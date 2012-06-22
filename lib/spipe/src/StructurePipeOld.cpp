/*
 * StructurePipe.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "StructurePipe.h"


// From PipelineLib
#include "SingleThreadedPipeline.h"

// Temporary includes ////////
#include <boost/any.hpp>


// Local includes
#include "blocks/EdgeDetect.h"
#include "blocks/LowestFreeEnergy.h"
#include "blocks/ParamPotentialGo.h"
#include "blocks/PotentialGo.h"
#include "blocks/PotentialParamSweep.h"
#include "blocks/RandomStructure.h"
#include "blocks/RemoveDuplicates.h"
#include "blocks/WriteStructure.h"
#include "blocks/NiggliReduction.h"
#include "common/SharedData.h"
#include "common/StructureData.h"
#include "common/UtilityFunctions.h"

#include <build_cell/AtomsDescription.h>
#include <build_cell/AtomGroupDescription.h>
#include <build_cell/DefaultCrystalGenerator.h>
#include <build_cell/Minsep.h>
#include <build_cell/StructureDescription.h>
#include <build_cell/RandomCellDescription.h>
#include <build_cell/RandomCellGenerator.h>
#include <common/AtomSpeciesId.h>
#include <io/ResReaderWriter.h>
#include <io/StructureWriterManager.h>
#include <potential/SimplePairPotential.h>
#include <potential/SimplePairPotentialData.h>
#include <potential/TpsdGeomOptimiser.h>
#include <utility/MultiIdx.h>
#include <utility/SortedDistanceComparator.h>
#include <utility/UniqueStructureSet.h>
#include <utility/PropertiesMap.h>

#include <Barrier.h>

#include <armadillo>

// MACROS ////////////////////////////////////

// NAMESPACES ////////////////////////////////

using sstbx::utility::PropertiesMap;
using sstbx::utility::Key;
using sstbx::utility::KeySupertype;

class SpecialisedProperties : public virtual PropertiesMap
{
public:

	using PropertiesMap::insert;
	using PropertiesMap::find;

	::std::pair<double * const, bool> insert(const Key<double> & key, double value);

	double * find(Key<double> & key);

	const double * find(const Key<double> & key) const;

	void erase(const Key<double> & key);

private:

	typedef ::std::map<const KeySupertype *, double> DoubleMap;

	typedef ::std::pair<const KeySupertype *, double> DoubleMapPair;

	DoubleMap myDoubleMap;
};

::std::pair<double * const, bool> SpecialisedProperties::insert(const Key<double> & key, double value)
{
	using ::std::pair;

	pair<DoubleMap::iterator, bool> result = myDoubleMap.insert(DoubleMapPair(&key, value));

	double * const out =	result.second ?
		&value :
		&result.first->second;

	return pair<double * const, bool>(out, result.second);
}

double * SpecialisedProperties::find(Key<double> & key)
{
	const DoubleMap::iterator it = myDoubleMap.find(&key);

	if(it == myDoubleMap.end())
		return NULL;

	return &it->second;
}

const double * SpecialisedProperties::find(const Key<double> & key) const
{
	const DoubleMap::const_iterator it = myDoubleMap.find(&key);

	if(it == myDoubleMap.end())
		return NULL;

	return &it->second;
}

void SpecialisedProperties::erase(const Key<double> & key)
{
	const DoubleMap::iterator it = myDoubleMap.find(&key);

	if(it == myDoubleMap.end())
		return;

	myDoubleMap.erase(it);
}

static Key<double> enthalpy;
static Key<int> timesFound;
static Key<size_t> timesFound2;
static Key<void *> somePointer;


int main(const int argc, const char * const argv[])
{
	using namespace ::spipe;
	using namespace arma;
	using namespace spipe::blocks;
	using namespace sstbx::build_cell;
	using namespace sstbx::potential;
	using namespace sstbx::utility;
	using pipelib::SingleThreadedPipeline;

	using ::sstbx::utility::MultiIdx;

	// TESTING
	using sstbx::utility::PropertiesMap;

	Key<int> myData;
	//PropertiesMap myMap;
	SpecialisedProperties myMap;
	myMap.insert(enthalpy, 10.0);
	myMap.insert(timesFound, 20);
	double val = *myMap.find(enthalpy);
	size_t val2 = *myMap.find(timesFound);
	myMap.insert(enthalpy, 1.0);
	myMap.erase(enthalpy);
	//myMap.insert(timesFound, (size_t)1);

	//double val3 = getValue(somePointer);

  // Get command line parameters
  if(argc != 7)
  {
    cout << "Usage: " << argv[0] << " [6 x params, format from+delta*nsteps]" << endl;
    return 0;
  }

	// Param sweep
	vec from(6), step(6);
	Col<unsigned int> steps(6);


  double lFrom, lStep;
  unsigned int lNSteps;
  bool parsedParams = true;
  for(size_t i = 0; i < 6; ++i)
  {
    if(spipe::common::parseParamString(argv[i + 1], lFrom, lStep, lNSteps))
    {
      from(i) = lFrom;
      step(i) = lStep;
      steps(i) = lNSteps;
    }
    else
    {
      parsedParams = false;
      cout << "Unable to parse parameter " << i << ": " << argv[i + 1] << endl;
      break;
    }
  }

  if(!parsedParams)
  {
    return 0;
  }

	// Create the pipeline
	SingleThreadedPipeline<StructureDataTyp, SharedDataTyp> parentPipe = SingleThreadedPipeline<StructureDataTyp, SharedDataTyp>();
	SingleThreadedPipeline<StructureDataTyp, SharedDataTyp> & pipe = parentPipe.spawnChild();

	// Describe the cell
  RandomCellDescription<> cellDesc;
	//cellDesc.setLatticeParams(1, 1, 1, 90, 90, 90);
  cellDesc.myMaxLengthRatio.reset(3.0);
  cellDesc.myMinAngle.reset(50);
  cellDesc.myMaxAngle.reset(140);
  cellDesc.myVolume.reset(25);
  cellDesc.myVolDelta.reset(0.2);

	// Describe the structure
	StructureDescription strDesc;
	AtomsDescription * const a1 = new AtomsDescription(sstbx::common::NA, 1);
	AtomsDescription * const a2 = new AtomsDescription(sstbx::common::CL, 1);
	strDesc.addChild(a1);
	strDesc.addChild(a2);
	Minsep * const minSep = new Minsep(1.5);
	strDesc.addAtomConstraint(minSep);

	// Set up random structure block
	RandomCellGenerator<> cellGenerator;
	DefaultCrystalGenerator crystalGenerator(cellGenerator);
	RandomStructure strBlock(100, crystalGenerator, &strDesc, &cellDesc);

	// Set up niggli reduction block
	NiggliReduction niggli;
	NiggliReduction niggli2;

	// Set up geometry optimisation block
	Mat<double> epsilon;
	epsilon.set_size(2, 2);
	epsilon << 1 << 2 << endr
			<< 2 << 2 << endr;

	Mat<double> sigma;
	sigma.set_size(2, 2);
	sigma << 2 << 2.5 << endr
			<< 2.5 << 2 << endr;

	Mat<int> beta;
	beta.set_size(2, 2);
	beta << -1 << 1 << endr
			<< 1 << -1 << endr;

  SimplePairPotential<> potential(2, epsilon, sigma, 2.5, beta, 12, 6, SimplePairPotential<>::NONE);
	TpsdGeomOptimiser<double, SimplePairPotential<double>::DataTyp > optimiser(potential);
	TpsdGeomOptimiser<double, SimplePairPotential<double>::DataTyp > optimiser2(potential);

	ParamPotentialGo go(potential, optimiser);

	// Set up the structure comparator
	SortedDistanceComparator comp;
	UniqueStructureSet<SortedDistanceComparator::DataTyp> sset(comp);
	RemoveDuplicates delDuplicates(sset);

	// Set up a writer block
	::sstbx::io::ResReaderWriter resRw;
	::sstbx::io::StructureWriterManager writeManager;
	writeManager.registerWriter(resRw);
	WriteStructure writer(writeManager);
	WriteStructure writer2(writeManager);

	// Lowest free energy filter
	LowestFreeEnergy lowest;

	// Set up the pipeline
	pipe.setStartBlock(strBlock);
	pipe.connect(strBlock, niggli);
	pipe.connect(niggli, go);
	//pipe.connect(go, niggli2);
	//pipe.connect(niggli2, delDuplicates);
	pipe.connect(go, delDuplicates);
	pipe.connect(delDuplicates, writer);
	// Put in a barrier
	::pipelib::Barrier<StructureDataTyp, SharedDataTyp> * barrier = pipe.createBarrier();
	pipe.connect(writer, *barrier);
	pipe.connect(*barrier, writer2);
	pipe.connect(writer2, lowest);

	using ::spipe::blocks::PotentialParamSweep;

	PotentialParamSweep sweep(from, step, steps, pipe);

	// Set up the parent pipeline


	// Edge detection
	SortedDistanceComparator edgeComparator;
	EdgeDetect<SortedDistanceComparator::DataTyp> edgeDetector(edgeComparator);

	parentPipe.setStartBlock(sweep);
	parentPipe.connect(sweep, edgeDetector);

	parentPipe.initialise();
	parentPipe.start();

	delete barrier;

	return 0;
}

