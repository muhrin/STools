/*
 * TranscodeGeneral.cpp
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "yaml/TranscodeGeneral.h"

#include <iomanip>
#include <sstream>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>

#include "utility/IndexingEnums.h"

// NAMESPACES ////////////////////////////////
namespace ssu = ::sstbx::utility;
namespace ssy = ::sstbx::yaml;

namespace YAML {

Node convert<arma::vec3>::encode(const arma::vec3 & rhs)
{
  using namespace ssu::cart_coords_enum;

  Node node;
  ::std::stringstream ss;
  ss << ::std::setprecision(12) << rhs(X) << " " << rhs(Y) << " " << rhs(Z);
  node = ss.str();
  return node;
}

bool convert<arma::vec3>::decode(const Node & node, arma::vec3 & rhs)
{
  using namespace ssu::cart_coords_enum;

  typedef ssy::VectorAsString<double> DoublesVec;
  // Maybe it is a string separated by spaces
  DoublesVec doublesVec;

  try
  {
    doublesVec = node.as<DoublesVec>();
  }
  catch(const YAML::TypedBadConversion<DoublesVec> & /*e*/)
  {
    return false;
  }

  if(doublesVec->size() != 3)
    return false; // Expecting 3 coordinates

  // Copy over values
  for(size_t i = X; i <= Z; ++i)
    rhs(i) = (*doublesVec)[i];

  return true;
}

Node convert<arma::vec>::encode(const arma::vec & rhs)
{
  Node node;

  if(rhs.n_rows == 0)
    return node;

  // Do first entry
  ::std::stringstream ss;
  ss << ::std::setprecision(12) << rhs(0);
  // and now rest
  for(size_t i = 1; i < rhs.n_rows; ++i)
  {
    ss << " " << rhs(i);
  }
  node = ss.str();
  return node;
}

bool convert<arma::vec>::decode(const Node& node, arma::vec & rhs)
{
  typedef ssy::VectorAsString<double> DoublesVec;
  // Maybe it is a string separated by spaces
  DoublesVec doublesVec;

  try
  {
    doublesVec = node.as<DoublesVec>();
  }
  catch(const YAML::TypedBadConversion<DoublesVec> & /*e*/)
  {
    return false;
  }

  // Copy over values
  rhs.set_size(doublesVec->size());
  for(size_t i = 0; i < doublesVec->size(); ++i)
    rhs(i) = (*doublesVec)[i];

  return true;
}

// Armadillo triangular matrices
Node convert<ssy::ArmaTriangularMat>::encode(
  const ssy::ArmaTriangularMat & rhs)
{
  Node node;
  ssy::VectorAsString<double> flatMatrix;

  for(size_t i = 0; i < rhs->n_rows; ++i)
  {
    for(size_t j = i; j < rhs->n_cols; ++j)
      flatMatrix->push_back((*rhs)(i, j));
  }

  node = flatMatrix;
  return node;
}

bool convert< ssy::ArmaTriangularMat>::decode(
  const Node & node, ssy::ArmaTriangularMat & rhs)
{
  typedef ssy::VectorAsString<double> DoublesVec;

  DoublesVec flatMatrix;
  try
  {
    flatMatrix = node.as<DoublesVec>();
  }
  catch(const YAML::TypedBadConversion<DoublesVec> & /*e*/)
  {
    return false;
  }

  if(flatMatrix->empty())
    return false;

  // Check if this is good for a triangular matrix
  const int nElements = flatMatrix->size();
  const double dSize = 0.5 * (-1.0 + ::std::sqrt(1.0 + 8.0 * nElements));
  const int iSize = static_cast<int>(dSize + 0.5);
  const int iTot = iSize * (iSize + 1) / 2;

  // Check that the double/int conversion hasn't caused problems
  if(iTot != nElements)
    return false;

  rhs->set_size(iSize, iSize);
  size_t k = 0;
  for(int i = 0; i < iSize; ++i)
  {
    for(int j = i; j < iSize; ++j)
    {
      (*rhs)(i, j) = (*flatMatrix)[k];
      ++k;
    }
  }
  (*rhs) = ::arma::symmatu(*rhs);

  return true;
}

}


