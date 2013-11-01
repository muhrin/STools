/*
 * sinfo.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include <vector>

#include <boost/program_options.hpp>

#include <armadillo>

#include <spl/utility/Armadillo.h>

// NAMESPACES ////////////////////////////////


// TYPEDEFS //////////////////////////////////

// CLASSES ///////////////////////////////////
struct InputOptions
{
  ::std::vector<double> inputParams;
};

// CONSTANTS /////////////////////////////////
static const int RESULT_SUCCESS = 0;
static const int RESULT_GENERAL_FAILURE = 1;

// FUNCTION DECLARATIONS ///////////////////
int processInputOptions(InputOptions & in, const int argc, char * argv[]);

int main(const int argc, char * argv[])
{
  // Process input and detect errors
  InputOptions in;
  int result = processInputOptions(in, argc, argv);
  if(result != RESULT_SUCCESS)
    return result;

  if(in.inputParams.empty())
  {
    ::std::cerr << "Error: no input parameters found" << ::std::endl;
    return RESULT_GENERAL_FAILURE;
  }

  ::arma::vec params(in.inputParams);

  ::boost::hash< ::arma::vec> vecHasher;
  ::std::cout << ::std::hex << vecHasher(params) << ::std::endl;

  return RESULT_SUCCESS;
}

int processInputOptions(InputOptions & in, const int argc, char * argv[])
{
  namespace po = ::boost::program_options;

  const ::std::string exeName(argv[0]);
  try
  {
    po::options_description desc("Usage: " + exeName + " [parameters list]...\n" +
      "\nOptions");
    desc.add_options()
      ("help", "Show help message")
      ("params,p", po::value< ::std::vector<double> >(&in.inputParams)->multitoken(), "parameters list")
    ;

    po::positional_options_description p;
    p.add("params", -1);

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

  return RESULT_SUCCESS;
}

