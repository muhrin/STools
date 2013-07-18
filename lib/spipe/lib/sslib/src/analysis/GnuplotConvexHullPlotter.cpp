/*
 * GnuplotConvexHullPlotter.cpp
 *
 *  Created on: Jul 17, 2013
 *      Author: Martin Uhrin
 */

#include "analysis/GnuplotConvexHullPlotter.h"

#ifdef SSLIB_USE_CGAL

#include <fstream>
#include <sstream>

#include <CGAL/Regular_complex_d.h>

#include "analysis/IConvexHullInfoSupplier.h"

namespace sstbx {
namespace analysis {

GnuplotConvexHullPlotter::GnuplotConvexHullPlotter()
{
  myOutputStem = "hull";
  myDrawBoundary = true;
  myDrawTieLines = false;
  mySupressEnergyDimension = false;
}

bool GnuplotConvexHullPlotter::outputHull(const ConvexHull & convexHull) const
{
  const ConvexHull::Hull * const hull = convexHull.getHull();
  if(!hull)
    return false;

  ::std::string str = myOutputStem + ".dat";
  ::std::ofstream datOut(str.c_str());
  str = myOutputStem + ".plt";
  ::std::ofstream pltOut(str.c_str());

  // Print the boundary
  if(myDrawBoundary)
    drawBoundary(pltOut, convexHull);

  // Now plot the points
  ConvexHull::PointD point;
  for(ConvexHull::Hull::Hull_vertex_const_iterator it = hull->hull_vertices_begin(),
      end = hull->hull_vertices_end(); it != end; ++it)
  {
    point = it->point();
    // Only print points below the 0 in energy, don't care about the top of the hull
    if(CGAL::to_double(point[0]) <= 0)
    {
      if(mySupressEnergyDimension)
        point[ConvexHull::CONVEX_PROPERTY_DIMENSION] = 0;

      // Make sure the point is the correct number of dimensions
      switch(convexHull.dims())
      {
      case 3:
      {
        if(mySupressEnergyDimension)
          point = convexHull.composition(point);
        break;
      }
      case 4:
        // Always have to suppress the 4th dimension as there's no way to plot it
        point = convexHull.composition(point);
        break;
      }
      if(mySupressEnergyDimension)
        point = convexHull.composition(point);

      datOut << printPoint(point) << ::std::endl;
    }
  }

  if(convexHull.dims() == 3 && !mySupressEnergyDimension)
    pltOut << "splot \"" << myOutputStem << ".dat\" u 2:3:1" << ::std::endl;
  else
  {
    if(!mySupressEnergyDimension)
      pltOut << "plot \"" << myOutputStem << ".dat\" u 2:1" << ::std::endl;
    else
      pltOut << "plot \"" << myOutputStem << ".dat\"" << ::std::endl;
  }

  drawTieLines(pltOut, convexHull);

  if(datOut.is_open())
    datOut.close();
  if(pltOut.is_open())
    pltOut.close();

  return true;
}

bool GnuplotConvexHullPlotter::outputHull(const ConvexHull & convexHull, const IConvexHullInfoSupplier & infoSupplier) const
{
  return false;
}

bool GnuplotConvexHullPlotter::getDrawTieLines() const
{
  return myDrawTieLines;
}

void GnuplotConvexHullPlotter::setDrawTieLines(const bool draw)
{
  myDrawTieLines = draw;
}

bool GnuplotConvexHullPlotter::getSupressEnergyDimension() const
{
  return mySupressEnergyDimension;
}

void GnuplotConvexHullPlotter::setSupressEnergyDimension(const bool supress)
{
  mySupressEnergyDimension = supress;
}

::std::string GnuplotConvexHullPlotter::printVec(const ConvexHull::VectorD & vec) const
{
  ::std::stringstream ss;
  ConvexHull::VectorD::Cartesian_const_iterator it = vec.cartesian_begin();
  const ConvexHull::VectorD::Cartesian_const_iterator end = vec.cartesian_end();

  // Print the first one as a special case
  if(it != end)
  {
    ss << CGAL::to_double(*it);
    ++it;
  }
  for(; it != end; ++it)
  {
    ss << "," << CGAL::to_double(*it);
  }

  return ss.str();
}

::std::string GnuplotConvexHullPlotter::printPoint(const ConvexHull::PointD & point) const
{
  ::std::stringstream ss;
  ConvexHull::PointD::Cartesian_const_iterator it = point.cartesian_begin();
  const ConvexHull::PointD::Cartesian_const_iterator end = point.cartesian_end();

  // Print the first one as a special case
  if(it != end)
  {
    ss << CGAL::to_double(*it);
    ++it;
  }
  for(; it != end; ++it)
  {
    ss << ", " << CGAL::to_double(*it);
  }

  return ss.str();
}


void GnuplotConvexHullPlotter::drawBoundary(::std::ostream & os, const ConvexHull & convexHull) const
{
  // Print the boundary
  int endpointLine = 1;
  ConvexHull::VectorD from, to;
  for(ConvexHull::EndpointsConstIterator it1 = convexHull.endpointsBegin(), end = convexHull.endpointsEnd();
      it1 != end; ++it1)
  {
    for(ConvexHull::EndpointsConstIterator it2 = it1 + 1; it2 != end; ++it2)
    {
      from = it1->second;
      to = it2->second;
      switch(convexHull.dims())
      {
      case 2:
      {
        from = ConvexHull::VectorD(2, from.cartesian_begin(), from.cartesian_end());
        to = ConvexHull::VectorD(2, to.cartesian_begin(), to.cartesian_end());
        break;
      }
      case 3:
      {
        if(!mySupressEnergyDimension)
        {
          from = ConvexHull::VectorD(3, from.cartesian_begin(), from.cartesian_end());
          to = ConvexHull::VectorD(3, to.cartesian_begin(), to.cartesian_end());
        }
        break;
      }
      }
      os << "set arrow " << endpointLine << " from "
          << printVec(from) << " to " << printVec(to)
          << " nohead linestyle 1" << ::std::endl;
      ++endpointLine;
    }
  }
}

void GnuplotConvexHullPlotter::drawTieLines(::std::ostream & os, const ConvexHull & convexHull) const
{
  const ConvexHull::Hull * const hull = convexHull.getHull();
  ConvexHull::PointD x0, x1;
  int line = 100;
//  for(ConvexHull::Hull::Facet_const_iterator facetIt = hull->facets_begin(),
//      end = hull->facets_end(); facetIt != end; ++facetIt)
//  {
//    ConvexHull::Hull::Point_const_iterator it = facetIt->points_begin();
//    const ConvexHull::PointD startPoint = *it;
//    x0 = startPoint;
//    for(ConvexHull::Hull::Point_const_iterator end = facetIt->points_end();
//        it != end; ++it)
//    {
//      x1 = *it;
//      os << "set arrow " << line++ << " from "
//          << printPoint(x0) << " to " << printPoint(x1)
//          << " nohead linestyle 1" << ::std::endl;
//      x0 = x1;
//    }
//    // Complete the facet
//    os << "set arrow " << line++ << " from "
//        << printPoint(x0) << " to " << printPoint(startPoint)
//        << " nohead linestyle 1" << ::std::endl;
//  }


  for(ConvexHull::Hull::Simplex_const_iterator simplexIt = hull->simplices_begin(),
      simplexEnd = hull->simplices_end(); simplexIt != simplexEnd; ++simplexIt)
  {
    const ConvexHull::PointD startPoint = hull->point_of_simplex(simplexIt, 0);
    x0 = startPoint;
    for(int i = 1; i <= hull->dimension(); ++i)
    {
      x1 = hull->point_of_simplex(simplexIt, i);
      os << "set arrow " << line++ << " from "
          << printPoint(x0) << " to " << printPoint(x1)
          << " nohead linestyle 1" << ::std::endl;
      x0 = x1;
    }
    // Complete the facet
    os << "set arrow " << line++ << " from "
        << printPoint(x0) << " to " << printPoint(startPoint)
        << " nohead linestyle 1" << ::std::endl;
  }
}

}
}

#endif // SSLIB_USE_CGAL
