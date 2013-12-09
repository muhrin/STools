/*
 * slock.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "STools.h"

#include <iostream>
#include <string>

#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/interprocess/sync/file_lock.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>

#include "utility/BoostCapabilities.h"

// MACROS ////////////////////////////////////

// NAMESPACES ////////////////////////////////
namespace fs = ::boost::filesystem;
namespace ip = ::boost::interprocess;

// CLASSES //////////////////////////////////
struct InputOptions
{
  bool tryLock;
  ::std::string lockFile;
};

// CONSTANTS /////////////////////////////////

// FUNCTIONS ////////////////////////////////
int
processCommandLineArgs(InputOptions & in, const int argc, char * argv[]);

int
main(const int argc, char * argv[])
{
  // Program options
  InputOptions in;

  int result = processCommandLineArgs(in, argc, argv);
  if(result != 0)
    return result;

  const fs::path fileToLock(in.lockFile);

  if(!fs::exists(fileToLock))
  {
    ::std::cerr << "Error: file " << in.lockFile << " does not exist.\n";
    return 1;
  }

  ip::file_lock lock(fileToLock.string().c_str());

  ::std::cout << "Locking " << fileToLock.string() << "..." << ::std::flush;
  if(in.tryLock)
  {
    if(!lock.try_lock())
      return 1;
  }
  else
    lock.lock();

  ::std::cout << "locked." << ::std::endl;
  while(true)
  {
    boost::this_thread::sleep(boost::posix_time::seconds(100));
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
        "slock\nUsage: " + exeName + " file_to_lock...\nOptions");
    general.add_options()
        ("help", "Show help message")
        ("try,t", po::value< bool>(&in.tryLock)->default_value(false),
        "Try locking the file, return immediately if unsuccessful")
        ("lock-file", po::value< ::std::string>(&in.lockFile)_ADD_REQUIRED_,
                "The file to lock")
        ;

    po::positional_options_description p;
    p.add("lock-file", 1);

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
