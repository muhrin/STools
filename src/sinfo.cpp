/*
 * sinfo.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "StructurePipe.h"

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>

//#include <armaillo>

// From Pipelib //


// From SSLib //
#include <common/Structure.h>
#include <common/Types.h>
#include <common/UnitCell.h>
#include <io/ResReaderWriter.h>

// My includes //

// NAMESPACES ////////////////////////////////
namespace fs = ::boost::filesystem;
namespace po = ::boost::program_options;
namespace ssu = ::sstbx::utility;
namespace ssc = ::sstbx::common;

const ::std::string VAR_PREFIX("@");

void printInfo(const ssc::Structure & structure, const ::std::string & infoString);


class TokenValue
{
public:

};

class InfoToken
{
public:

  InfoToken(const ::std::string & token);
  virtual ~InfoToken() {}


private:

  ::std::string myToken;

};

int main(const int argc, char * argv[])
{

  const ::std::string exeName(argv[0]);

  // Input options
  ::std::vector< ::std::string> inputFiles;
  ::std::string infoString;

  try
  {
    po::options_description desc("Usage: " + exeName + " [options] files...\nOptions:");
    desc.add_options()
      ("help", "Show help message")
      ("info-string,i", po::value< ::std::string>(&infoString)->default_value(VAR_PREFIX + "v \n"), "info string")
      ("input-file", po::value< ::std::vector< ::std::string> >(&inputFiles)->required(), "input file(s)")
    ;

    po::positional_options_description p;
    p.add("input-file", -1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);

    // Deal with help first, otherwise missing required parameters will cause exception
    if(vm.count("help"))
    {
      ::std::cout << desc << ::std::endl;
      return 1;
    }

    po::notify(vm);
  }
  catch(std::exception& e)
  {
    ::std::cout << e.what() << "\n";
    return 1;
  }


  sstbx::io::ResReaderWriter resReader;


  ::std::string inputFile;
  fs::path structurePath;
  ssc::StructurePtr structure;
  BOOST_FOREACH(inputFile, inputFiles)
  {
    fs::path structurePath(inputFile);
    structure = resReader.readStructure(structurePath);

    printInfo(*structure.get(), infoString);
  }

  return 0;
}

void printInfo(const ssc::Structure & structure, const ::std::string & infoString)
{
  ::std::string printString;

  size_t pos = 0, currentPos = 0;
  size_t nextWhitespacePos;
  ::std::string infoSymbol;
  while((pos = infoString.find(VAR_PREFIX, currentPos)) != ::std::string::npos)
  {
    printString += infoString.substr(currentPos, pos);
    nextWhitespacePos = infoString.find(" ", pos);

    currentPos += 1;
    if(nextWhitespacePos != ::std::string::npos)
    {
      infoSymbol = infoString.substr(pos + 1, nextWhitespacePos - pos - 1);
      if(infoSymbol == "v")
      {
        printString += ::boost::lexical_cast< ::std::string>(structure.getUnitCell()->getVolume());
        currentPos += 1;
      }
    }
  }
  printString += "\n";

  ::std::cout << printString;
}
