/*
 * StructureDiff.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "StructurePipe.h"

#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <armadillo>

// From Pipelib //


// From SSLib //
#include <common/Structure.h>
#include <io/ResReaderWriter.h>
#include <utility/SortedDistanceComparator.h>

// My includes //


// NAMESPACES ////////////////////////////////

int main(const int argc, const char * const argv[])
{
  using boost::filesystem::path;
  using std::cout;
  using std::endl;
  using sstbx::common::Structure;

  if(argc != 4)
  {
    cout << "Usage: " << argv[0] << " <tolerance> <structure1> <structure2>" << endl;
    return 0;
  }

  // Get the tolerance
  double tolerance = 0.0;
  try
  {
    tolerance = boost::lexical_cast<double>(argv[1]);
  }
  catch(const boost::bad_lexical_cast & e)
  {
    cout << argv[1] << ": " << e.what() << endl;
    return 0;
  }

  // Get the structure paths
  boost::filesystem::path path1(argv[2]), path2(argv[3]);

  if(!boost::filesystem::exists(path1))
  {
    std::cerr << "File " << path1 << " does not exist." << endl;
    path1 = path1.string() + ".res";
    std::cerr << "Trying " << path1;
    if(!boost::filesystem::exists(path1))
    {
      std::cerr << "File " << path1 << " does not exist." << endl;
      return 0;
    }
  }
  if(!boost::filesystem::exists(path2))
  {
    std::cerr << "File " << path2 << " does not exist." << endl;
    path2 = path2.string() + ".res";
    std::cerr << "Trying " << path2;
    if(!boost::filesystem::exists(path2))
    {
      std::cerr << "File " << path2 << " does not exist." << endl;
      return 0;
    }
  }

  Structure str1, str2;

  sstbx::io::ResReaderWriter resReader;

  // Load the structures from file
  resReader.readStructure(str1, path1);
  resReader.readStructure(str2, path2);

  // Niggli reduce both unit cells
  str1.getUnitCell()->niggliReduce();
  str2.getUnitCell()->niggliReduce();

  //resReader.writeStructure(str1, boost::filesystem::path("nig1.res"));
  //resReader.writeStructure(str2, boost::filesystem::path("nig22.res"));

  // Get the comparator
  sstbx::utility::SortedDistanceComparator comparator(tolerance);

  double diff = comparator.compareStructures(str1, str2);

  if(diff < tolerance)
  {
    cout << "TRUE" << endl;
  }
  else
  {
    cout << "FALSE " << diff << endl;
  }

	return 0;
}
