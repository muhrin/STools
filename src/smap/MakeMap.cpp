/*
 * MakeMap.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "StructurePipe.h"

#include <boost/filesystem.hpp>

#include <armadillo>

// From Pipelib //
#include <SingleThreadedPipeline.h>

 //
#include <spl/potential/SimplePairPotential.h>
#include <spl/utility/SortedDistanceComparator.h>

// My includes //
#include "blocks/EdgeDetect.h"
#include "blocks/LoadPotStructures.h"
#include "spl/common/SharedData.h"
#include "spl/common/StructureData.h"

// NAMESPACES ////////////////////////////////

int main()
{
  using namespace pipelib;
  using namespace spipe;
  using namespace spipe::blocks;
  using arma::Mat;
  using arma::endr;
  using boost::filesystem::path;

  path potFilePath("simple_pair_potential.dat");

  SingleThreadedPipeline<StructureDataTyp, SharedDataTyp> pipe;

  // Setup blocks
  // Load structures
	Mat<double> epsilon;
	epsilon.set_size(2, 2);
	epsilon << 1 << 2 << endr
			<< 2 << 2 << endr;

	Mat<double> sigma;
	sigma.set_size(2, 2);
	sigma << 2 << 2.5 << endr
			<< 2.5 << 2 << endr;

	Mat<double> beta;
	beta.set_size(2, 2);
	beta << -1 << 1 << endr
			<< 1 << -1 << endr;

	// Param sweep
  arma::vec from(6), step(6);
  arma::Col<unsigned int> steps(6);
	from
    << 1 << endr // Epsilon
    << 2 << endr
    << 2 << endr
    << 2 << endr // Sigma
    << 0.05 << endr
    << 0.05 << endr;
	step
    << 0 << endr // Epsilon
    << 0 << endr
    << 0 << endr
    << 0 << endr // Sigma
    << 0.05 << endr
    << 0.05 << endr;
	steps
    << 0 << endr // Epsilon
    << 0 << endr
    << 0 << endr
    << 0 << endr // Sigma
    << 81 << endr
    << 81 << endr;

  SharedDataTyp & sharedDat = pipe.getSharedData();
  sharedDat.potSweepFrom.reset(from);
  sharedDat.potSweepStep.reset(step);
  sharedDat.potSweepNSteps.reset(steps);

  spl::potential::SimplePairPotential<> pot(2, epsilon, sigma, 2.5, beta, 12, 6);
  LoadPotStructures loadStructures(pot, potFilePath, true);


  // Edge detect
  spl::utility::SortedDistanceComparator comparator;
  EdgeDetect edgeDetect(comparator);


  // Build the pipeline
  pipe.setStartBlock(loadStructures);
  pipe.connect(loadStructures, edgeDetect);

  // Start the pipeline
  pipe.initialise();
  pipe.start();

	return 0;
}

