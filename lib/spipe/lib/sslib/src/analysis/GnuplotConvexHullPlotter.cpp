/*
 * GnuplotConvexHullPlotter.cpp
 *
 *  Created on: Jul 17, 2013
 *      Author: Martin Uhrin
 */

#include "analysis/GnuplotConvexHullPlotter.h"

#ifdef SSLIB_USE_CGAL

//#define DEBUG_GNUPLOT_CONVEX_HULL_PLOTTER

#include <fstream>
#include <sstream>

#include <boost/optional.hpp>

#include <CGAL/Regular_complex_d.h>

#include "analysis/IConvexHullInfoSupplier.h"

namespace sstbx {
namespace analysis {

GnuplotConvexHullPlotter::GnuplotConvexHullPlotter()
{
  myOutputStem = "hull";
  myDrawBoundary = true;
  myDrawTieLines = false;
  myDrawLabels = true;
  myDrawOffHullPoints = true;
  mySupressEnergyDimension = false;
}

bool
GnuplotConvexHullPlotter::outputHull(const ConvexHull & convexHull) const
{
  return outputHull(convexHull, NULL);
}

bool
GnuplotConvexHullPlotter::outputHull(const ConvexHull & convexHull,
    const IConvexHullInfoSupplier * const infoSupplier) const
{
  const ConvexHull::Hull * const hull = convexHull.getHull();
  if(!hull)
    return false;

  ::std::string str = myOutputStem + ".dat";
  ::std::ofstream datOut(str.c_str());
  str = myOutputStem + ".plt";
  ::std::ofstream pltOut(str.c_str());

  Plot plot;

  // Set up the graph style
  setStyles(pltOut, convexHull);

  // Print the boundary
  if(myDrawBoundary)
    drawBoundary(pltOut, convexHull, plot);

  drawEndpointLabels(pltOut, convexHull, plot);
  if(myDrawTieLines)
    drawTieLines(pltOut, convexHull, plot);

  // Now plot the points
  ConvexHull::PointD point;
  for(ConvexHull::Hull::Hull_vertex_const_iterator it =
      hull->hull_vertices_begin(), end = hull->hull_vertices_end(); it != end;
      ++it)
  {
    point = it->point();
    const ConvexHull::PointId pointId = point.getId();

    point = prepPoint(point);

    datOut << printPoint(point) << ::std::endl;
    if(infoSupplier && myDrawLabels)
    {
      ::std::vector< ConvexHull::HullTraits::FT> labelPt(
          point.cartesian_begin(), point.cartesian_end());
      labelPt[labelPt.size() - 1] -= LABEL_MARGIN;
      const ::std::string & label = infoSupplier->getLabel(convexHull,
          pointId);
      if(!label.empty())
        pltOut
            << plot.drawLabel(label,
                ConvexHull::PointD(labelPt.size(), labelPt.begin(),
                    labelPt.end()));
    }
  }

  bool haveOffHullPoints = false;
  if(myDrawOffHullPoints)
  {
    ::boost::optional< bool> stable;
    // Start a new data set
    datOut << ::std::endl << ::std::endl;
    for(ConvexHull::EntriesConstIterator it = convexHull.entriesBegin(), end =
        convexHull.entriesEnd(); it != end; ++it)
    {
      const ConvexHull::PointD & point = *(it->getPoint());
      stable = convexHull.isStable(point);
      if(stable && !*stable)
      {
        datOut << printPoint(prepPoint(point)) << ::std::endl;
        haveOffHullPoints = true;
      }
    }
  }

  if(plotDims(convexHull) == 3 || convexHull.dims() == 3)
    pltOut << "set size square" << ::std::endl;

  ::std::stringstream plotStream;
  plotStream << "\"" << myOutputStem << ".dat\" i 0 ls "
      << HULL_POINT_LINE_STYLE << " title \"Hull point\"";
  if(myDrawOffHullPoints && haveOffHullPoints)
    plotStream << ", \"" << myOutputStem << ".dat\" i 1 ls "
        << OFF_HULL_LINE_STYLE << " title \"Off hull point\"";

  if(plotDims(convexHull) == 3)
    pltOut << "splot " << plotStream.str() << ::std::endl;
  else
    pltOut << "plot " << plotStream.str() << ::std::endl;

  if(datOut.is_open())
    datOut.close();
  if(pltOut.is_open())
    pltOut.close();

  return true;
}

bool
GnuplotConvexHullPlotter::getDrawTieLines() const
{
  return myDrawTieLines;
}

void
GnuplotConvexHullPlotter::setDrawTieLines(const bool draw)
{
  myDrawTieLines = draw;
}

bool
GnuplotConvexHullPlotter::getSupressEnergyDimension() const
{
  return mySupressEnergyDimension;
}

void
GnuplotConvexHullPlotter::setSupressEnergyDimension(const bool supress)
{
  mySupressEnergyDimension = supress;
}

void
GnuplotConvexHullPlotter::setDrawHullLabels(const bool label)
{
  myDrawLabels = label;
}

void
GnuplotConvexHullPlotter::setDrawOffHullPoints(const bool draw)
{
  myDrawOffHullPoints = draw;
}

const double GnuplotConvexHullPlotter::LABEL_MARGIN = 0.05;
const int GnuplotConvexHullPlotter::BOUNDARY_LINE_STYLE = 1;
const int GnuplotConvexHullPlotter::HULL_POINT_LINE_STYLE = 2;
const int GnuplotConvexHullPlotter::OFF_HULL_LINE_STYLE = 3;

GnuplotConvexHullPlotter::Plot::Plot() :
    myArrowCounter(0), myLabelCounter(0)
{
}

::std::string
GnuplotConvexHullPlotter::Plot::drawLine(const ConvexHull::PointD & x0,
    const ConvexHull::PointD & x1)
{
  return drawLine(x0, x1, 1);
}

::std::string
GnuplotConvexHullPlotter::Plot::drawLine(const ConvexHull::PointD & x0,
    const ConvexHull::PointD & x1, const int lineStyle)
{
  ::std::stringstream ss;
  ss << "set arrow " << ++myArrowCounter << " from " << printPoint(x0) << " to "
      << printPoint(x1) << " nohead linestyle " << lineStyle << ::std::endl;
  return ss.str();
}

::std::string
GnuplotConvexHullPlotter::Plot::drawLabel(const ::std::string label,
    const ConvexHull::PointD & x) const
{
  ::std::stringstream ss;
  ss << "set label \"" << label << "\" at " << printPoint(x) << " centre"
      << ::std::endl;
  return ss.str();
}

::std::string
GnuplotConvexHullPlotter::Plot::printPoint(
    const ConvexHull::PointD & point) const
{
  ::std::stringstream ss;
  ConvexHull::PointD::Cartesian_const_iterator it = point.cartesian_begin();
  const ConvexHull::PointD::Cartesian_const_iterator end =
      point.cartesian_end();

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

::std::string
GnuplotConvexHullPlotter::printPoint(const ConvexHull::PointD & point) const
{
  ::std::stringstream ss;
  ConvexHull::PointD::Cartesian_const_iterator it = point.cartesian_begin();
  const ConvexHull::PointD::Cartesian_const_iterator end =
      point.cartesian_end();

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

void
GnuplotConvexHullPlotter::setStyles(::std::ostream & os,
    const ConvexHull & convexHull) const
{
  os << "set xtics nomirror" << ::std::endl;
  os << "set ytics nomirror" << ::std::endl;

  os << "set style line " << BOUNDARY_LINE_STYLE
      << " ps 1.5 pt 7 lc rgb '#000000'" << ::std::endl;
  os << "set style line " << HULL_POINT_LINE_STYLE
      << " ps 1.5 pt 7 lc rgb '#dd181f'" << ::std::endl;
  os << "set style line " << OFF_HULL_LINE_STYLE
      << " ps 0.6 pt 7 lc rgb '#0060ad'" << ::std::endl;

  if(convexHull.dims() > 2)
  {
    os << "unset border" << ::std::endl;
    os << "unset ytics" << ::std::endl;
    os << "set yrange[0:1]" << ::std::endl;
    if(convexHull.dims() > 3)
    {
      os << "unset ztics" << ::std::endl;
      os << "set zrange[0:1]" << ::std::endl;
    }
  }

  // Axis label
  if(!mySupressEnergyDimension && convexHull.dims() != 4)
  {
    if(plotDims(convexHull) == 2)
      os << "set ylabel \"Formation energy\"" << ::std::endl;
    else
      os << "set zlabel \"Formation energy\" rotate by 90" << ::std::endl;
  }
}

void
GnuplotConvexHullPlotter::drawBoundary(::std::ostream & os,
    const ConvexHull & convexHull, Plot & plot) const
{
  // With only two dimensions the x axis will act as the boundary
  if(convexHull.dims() == 2)
    return;

  // Print the boundary
  ConvexHull::PointD from, to;
  for(ConvexHull::EndpointsConstIterator it1 = convexHull.endpointsBegin(),
      end = convexHull.endpointsEnd(); it1 != end; ++it1)
  {
    for(ConvexHull::EndpointsConstIterator it2 = it1 + 1; it2 != end; ++it2)
    {
      from = it1->second;
      to = it2->second;
      os << plot.drawLine(prepPoint(from), prepPoint(to), BOUNDARY_LINE_STYLE);
    }
  }
}

void
GnuplotConvexHullPlotter::drawTieLines(::std::ostream & os,
    const ConvexHull & convexHull, Plot & plot) const
{
  const ConvexHull::Hull * const hull = convexHull.getHull();
  ConvexHull::PointD x0, x1;
  for(ConvexHull::Hull::Facet_const_iterator facetIt = hull->facets_begin(),
      end = hull->facets_end(); facetIt != end; ++facetIt)
  {
    // Don't want facets that are vertical w.r.t. the convex dimension
    if(convexHull.isVerticalFacet(facetIt))
      continue;

#ifdef DEBUG_GNUPLOT_CONVEX_HULL_PLOTTER
    os << "# Start of facet\n";
#endif

    const ConvexHull::PointD startPoint = prepPoint(hull->point_of_facet(facetIt, 0));
    x0 = x1 = startPoint;
    for(int i = 0; i < hull->current_dimension() - 1; ++i)
    {
      x0 = prepPoint(hull->point_of_facet(facetIt, i));
      x1 = prepPoint(hull->point_of_facet(facetIt, i + 1));

      os << plot.drawLine(x0, x1);
    }
    // Complete the facet
    os << plot.drawLine(x1, startPoint);

#ifdef DEBUG_GNUPLOT_CONVEX_HULL_PLOTTER
    os << "# End of facet\n";
#endif
  }
}

void
GnuplotConvexHullPlotter::drawEndpointLabels(::std::ostream & os,
    const ConvexHull & convexHull, Plot & plot) const
{
  const int plotDims = this->plotDims(convexHull);

  // Print the boundary
  ConvexHull::PointD from, to;
  for(ConvexHull::EndpointsConstIterator it1 = convexHull.endpointsBegin(),
      end = convexHull.endpointsEnd(); it1 != end; ++it1)
  {
    ConvexHull::VectorD vec(convexHull.dims());
    for(ConvexHull::EndpointsConstIterator it2 = convexHull.endpointsBegin();
        it2 != end; ++it2)
    {
      if(it1 != it2)
        vec += (it1->second - CGAL::ORIGIN) - (it2->second - CGAL::ORIGIN);
    }
    vec *= LABEL_MARGIN / CGAL::sqrt(vec.squared_length());
    vec += it1->second - CGAL::ORIGIN;
    os
        << plot.drawLabel(it1->first.toString(),
            ConvexHull::PointD(plotDims, vec.cartesian_begin(),
                vec.cartesian_begin() + plotDims));
  }
}

int
GnuplotConvexHullPlotter::plotDims(const ConvexHull & convexHull) const
{
  return
      mySupressEnergyDimension || convexHull.dims() == 4 ?
          convexHull.dims() - 1 : convexHull.dims();
}

ConvexHull::PointD
GnuplotConvexHullPlotter::prepPoint(const ConvexHull::PointD & point) const
{
  ::std::vector< ConvexHull::HullTraits::FT> coords(point.cartesian_begin(),
      point.cartesian_end());

  if(mySupressEnergyDimension)
    coords[coords.size() - 1] = 0;

  // Make sure the point is the correct number of dimensions
  switch (coords.size())
  {
  case 3:
  {
    if(mySupressEnergyDimension)
      coords.erase(coords.end() - 1);
    break;
  }
  case 4:
    // Always have to suppress the 4th dimension as there's no way to plot it
    coords.erase(coords.end() - 1);
    break;
  }
  return ConvexHull::PointD(coords.size(), coords.begin(), coords.end());
}

}
}

#endif // SSLIB_USE_CGAL
