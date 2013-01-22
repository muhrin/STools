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

#include "OptionalTypes.h"
#include "build_cell/IFragmentGenerator.h"
#include "build_cell/Sphere.h"

// FORWARD DECLARES //////////////////////////

namespace sstbx {
namespace build_cell {
class AtomsDescription;

class AtomsGenerator : public IFragmentGenerator
{
  typedef IFragmentGenerator::GenerationTicketId GenerationTicketId;
  typedef ::std::vector<AtomsDescription> Atoms;
public:
  typedef IFragmentGenerator::GenerationTicket GenerationTicket;
  typedef Atoms::iterator iterator;
  typedef Atoms::const_iterator const_iterator;
  typedef ::boost::optional<Sphere> OptionalSphere;

  AtomsGenerator() {}
  AtomsGenerator(const AtomsGenerator & toCopy);

  size_t numAtoms() const;
  iterator beginAtoms();
  const_iterator beginAtoms() const;
  iterator endAtoms();
  const_iterator endAtoms() const;

  iterator addAtoms(const AtomsDescription & atoms);
  void eraseAtoms(iterator pos);

  const OptionalSphere & getGenerationSphere() const;
  void setGenerationSphere(const OptionalSphere & shere);
  
  // From IFragmentGenerator ////////
  virtual GenerationOutcome generateFragment(
    StructureBuild & build,
    const GenerationTicket ticket,
    const common::AtomSpeciesDatabase & speciesDb
  ) const;

  virtual GenerationTicket getTicket() const;
  virtual StructureContents getGenerationContents(
    const GenerationTicket ticket,
    const common::AtomSpeciesDatabase & speciesDb
  ) const;

  virtual void handleReleased(const GenerationTicketId & id);

  virtual IFragmentGeneratorPtr clone() const;
  // End from IFragmentGenerator

private:

  typedef ::std::pair< ::arma::vec3, bool> AtomPosition;

  AtomPosition generatePosition(const AtomsDescription & atom, const StructureBuild & build) const;
  double getRadius(const AtomsDescription & atom, const common::AtomSpeciesDatabase & speciesDb) const;

  Atoms myAtoms;
  OptionalSphere myGenerationSphere;
  
};

}
}

#endif /* ATOMS_GENERATOR_H */
