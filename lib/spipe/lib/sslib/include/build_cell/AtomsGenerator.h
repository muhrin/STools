/*
 * AtomsGenerator.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef ATOMS_GENERATOR_H
#define ATOMS_GENERATOR_H

// INCLUDES /////////////////////////////////

#include "SSLib.h"

#include <boost/noncopyable.hpp>
#include <boost/scoped_ptr.hpp>

#include "OptionalTypes.h"
#include "build_cell/IFragmentGenerator.h"
#include "build_cell/IGeneratorShape.h"
#include "build_cell/SymmetryGroup.h"

// FORWARD DECLARES //////////////////////////

namespace sstbx {
namespace build_cell {
class AtomsDescription;
class BuildAtomInfo;
class AtomsGeneratorConstructionInfo;

// TODO: Put in construction info object so that AtomsGenerator is immutable


class AtomsGenerator : public IFragmentGenerator, ::boost::noncopyable
{
  typedef IFragmentGenerator::GenerationTicketId GenerationTicketId;
  typedef ::std::vector<AtomsDescription> Atoms;
public:
  typedef IFragmentGenerator::GenerationTicket GenerationTicket;
  typedef Atoms::iterator iterator;
  typedef Atoms::const_iterator const_iterator;
  typedef UniquePtr<IGeneratorShape>::Type GenShapePtr;

  struct TransformSettings
  {
    enum Value
    {
      FIXED         = 0x00, // 000
      RAND_POS      = 0x01, // 001
      RAND_ROT_DIR  = 0x02, // 010
      RAND_ROT_ANGLE= 0x04  // 100
    };
  };

  AtomsGenerator(AtomsGeneratorConstructionInfo & constructionInfo);
  AtomsGenerator(const AtomsGenerator & toCopy);

  size_t numAtoms() const;
  iterator beginAtoms();
  const_iterator beginAtoms() const;
  iterator endAtoms();
  const_iterator endAtoms() const;

  iterator addAtoms(const AtomsDescription & atoms);
  void eraseAtoms(iterator pos);

  const IGeneratorShape * getGeneratorShape() const;
  
  // From IFragmentGenerator ////////
  virtual GenerationOutcome generateFragment(
    StructureBuild & build,
    const GenerationTicket ticket,
    const common::AtomSpeciesDatabase & speciesDb
  ) const;

  virtual GenerationTicket getTicket();
  virtual StructureContents getGenerationContents(
    const GenerationTicket ticket,
    const common::AtomSpeciesDatabase & speciesDb
  ) const;

  virtual void handleReleased(const GenerationTicketId & id);

  virtual IFragmentGeneratorPtr clone() const;
  // End from IFragmentGenerator

private:
  typedef ::std::pair< ::arma::vec3, bool> AtomPosition;
  typedef ::std::map<const AtomsDescription *, int> AtomCounts;

  struct GenerationInfo
  {
    AtomCounts atomCounts;
    ::arma::mat44 shapeTransform;
  };

  typedef ::std::map<GenerationTicket::IdType, GenerationInfo> TicketsMap;

  AtomPosition generatePosition(
    BuildAtomInfo & atomInfo,
    const AtomsDescription & atom,
    const StructureBuild & build,
    const unsigned int multiplicity,
    const ::arma::mat44 & transformation
  ) const;
  bool generateSpecialPosition(
    ::arma::vec3 & posOut,
    SymmetryGroup::OpMask & opMaskOut,
    const SymmetryGroup::EigenspacesAndMasks & spaces,
    const IGeneratorShape & genShape,
    const ::arma::mat44 & transformation
  ) const;
  OptionalArmaVec3 generateSpeciesPosition(
    const SymmetryGroup::Eigenspace & eigenspace,
    const IGeneratorShape & genShape,
    const ::arma::mat44 & transformation
  ) const;
  double getRadius(const AtomsDescription & atom, const common::AtomSpeciesDatabase & speciesDb) const;

  const IGeneratorShape & getGenShape(const StructureBuild & build) const;

  ::arma::mat44 generateTransform(const StructureBuild & build) const;

  Atoms myAtoms;
  const ::boost::scoped_ptr<IGeneratorShape> myGenShape;
  int myTransformMask;
  const unsigned int myNumReplicas;
  ::arma::vec3 myTranslation;
  ::arma::vec4 myRotation;
  TicketsMap myTickets;
  GenerationTicket::IdType myLastTicketId;
};

struct AtomsGeneratorConstructionInfo
{
  AtomsGeneratorConstructionInfo():
  numReplicas(1),
  transformMask(AtomsGenerator::TransformSettings::FIXED)
  {}

  int numReplicas;
  UniquePtr<IGeneratorShape>::Type genShape;
  int transformMask;
  OptionalArmaVec3 pos;
  OptionalArmaVec4 rot;
};

}
}

#endif /* ATOMS_GENERATOR_H */
