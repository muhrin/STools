/*
 * Ancillary.h
 *
 *
 *  Created on: Nov 19, 2012
 *      Author: Martin Uhrin
 */

#ifndef SINFO_ANCILLARY_H
#define SINFO_ANCILLARY_H

// INCLUDES /////////////////////////////////////////////
#include <functional>
#include <string>
#include <vector>

#include <boost/ptr_container/ptr_map.hpp>
#include <boost/range/iterator_range.hpp>

// From SSTbx
#include <common/AtomsFormula.h>
#include <common/Structure.h>
#include <utility/TypedDataTable.h>

// FORWARD DECLARES ////////////////////////////////
namespace sstbx {
namespace common {
class Structure;
}
}

namespace stools {
namespace utility {
class EnergyToken;
class InfoToken;
template <typename T>
class RelativeValueToken;
}

namespace sinfo {

// TYPEDEFS /////////////////////////////////////
typedef ::std::vector< ::std::string> InfoStringTokens;
typedef ::boost::ptr_map< ::std::string, utility::InfoToken> TokensMap;
typedef ::std::auto_ptr<utility::InfoToken> TokenPtr;
typedef ::sstbx::utility::TypedDataTable<const ::sstbx::common::Structure *> StructureInfoTable;
typedef StructureInfoTable::SortedKeys SortedKeys;

// CONSTATNS ////////////////////////////////////
extern const ::std::string VAR_BRACKET;
extern const ::std::string VAR_FORMAT;
extern const ::std::string DEFAULT_INFO_STRING;
extern const ::std::string INFO_STRING_AUTO;
const int PRINT_ALL = -1;
extern const double MAX_HULL_DIST_IGNORE;

// ENUMS /////////////////////////////////////////
struct Result
{
  enum Value {SUCCESS = 0, FAILURE};
};

// CLASSES //////////////////////////////////////
struct InputOptions
{
  ::std::vector< ::std::string> inputFiles;
  ::std::string infoString;
  ::std::vector< ::std::string> additionalTokens;
  ::std::string sortToken;
  bool reverseSortComparison;
  ::std::string emptyString;
  bool freeMode;
  bool noHeader;
  bool recursive;
  bool summary;
  int printTop; // Print top n structures (-1 for all)
  bool uniqueMode;
  double uniqueTolerance;
  ::std::string filterString;
  int compositionTop;
  double maxHullDist;
  bool stableCompositions;
};

struct CustomisableTokens
{
  CustomisableTokens()
  {
    lowestEnergy = NULL;
    lowestEnergyPerAtom = NULL;
    lowestEnthalpy = NULL;
    lowestEnthalpyPerAtom = NULL;
  }
  utility::RelativeValueToken<double> * lowestEnergy;
  utility::RelativeValueToken<double> * lowestEnergyPerAtom;
  utility::RelativeValueToken<double> * lowestEnthalpy;
  utility::RelativeValueToken<double> * lowestEnthalpyPerAtom;
};

struct TokensInfo
{
  ::std::vector<const utility::InfoToken *> tokens;
  const utility::InfoToken * sortToken;
  ::std::vector< ::std::string> formatStrings;
};

class FormulaFilter : public ::std::unary_function< const ::sstbx::common::Structure &, bool>
{
public:
  FormulaFilter(const ::sstbx::common::AtomsFormula & formula) :
      myFormula(formula)
  {
  }

  bool
  operator ()(::sstbx::common::Structure & structure)
  {
    return !myFormula.isEmpty()
        && structure.getComposition().numMultiples(myFormula) == 0;
  }
private:
  const ::sstbx::common::AtomsFormula myFormula;
};

// FUNCTIONS /////////////////////////////////////

Result::Value
processInputOptions(InputOptions & in, const int argc, char * argv[], const TokensMap & tokensMap);

void
parseAdditionalTokens(InputOptions & in);

CustomisableTokens generateTokens(TokensMap & map);

Result::Value
getRequiredTokens(
  TokensInfo & tokensInfo,
  const TokensMap & tokensMap,
  const InputOptions & in);

void printInfo(
  const StructureInfoTable & infoTable,
  const SortedKeys & orderedKeys,
  const TokensInfo & tokensInfo,
  const InputOptions & in,
  const size_t numToPrint);

}
}


#endif /* SINFO_ANCILLARY_H */
