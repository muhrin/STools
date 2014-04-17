/*
 * GnuplotAnchorArrangementPlotter.h
 *
 *  Created on: Nov 11, 2013
 *      Author: Martin Uhrin
 */

#ifndef GNUPLOT_ANCHOR_ARRANGEMENT_PLOTTER_DETAIL_H
#define GNUPLOT_ANCHOR_ARRANGEMENT_PLOTTER_DETAIL_H

// INCLUDES ///////////////////
#include <fstream>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/range/iterator_range.hpp>

#include <CGAL/centroid.h>

#include <spl/analysis/AnchorArrangement.h>
#include <spl/math/LinearAlgebra.h>

// FORWARD DECLARATIONS ///////

// DEFINITION ///////////////////////

namespace spl {
namespace analysis {

template< typename Map>
  GnuplotAnchorArrangementPlotter< Map>::GnuplotAnchorArrangementPlotter(
      const std::string & outputFileStem) :
      myOutputFileStem(outputFileStem)
  {
  }

template< typename Map>
  bool
  GnuplotAnchorArrangementPlotter< Map>::outputArrangement(
      const Arrangement & arrangement) const
  {
    typedef CGAL::Point_2< typename Map::Kernel> MapPoint;

    using CGAL::to_double;

    const std::string edgesOutStr = myOutputFileStem + ".edge", pltOutStr =
        myOutputFileStem + ".plt", polysOutStr = myOutputFileStem + ".poly";

    std::ofstream edgesOut(edgesOutStr.c_str());
    std::ofstream pltOut(pltOutStr.c_str());
    std::ofstream polysOut(polysOutStr.c_str());

    std::stringstream plotCommand;
    plotCommand << "plot ";

    size_t polyIdx = 0;
    MapPoint centre;
    BOOST_FOREACH(const typename Arrangement::Face & face,
        boost::make_iterator_range(arrangement.faces_begin(),
            arrangement.faces_end()))
    {
      if(!face.is_unbounded() && !face.is_fictitious())
      {
        const FacePolygon poly = getFacePolygon(face);
        centre = CGAL::centroid(poly.vertices_begin(), poly.vertices_end(),
            CGAL::Dimension_tag< 0>());
        if(face.data().label)
        {
          pltOut << "set label \"" << *face.data().label << "\" at "
              << CGAL::to_double(centre.x()) << ", "
              << CGAL::to_double(centre.y()) << " centre front\n";
        }

        BOOST_FOREACH(const MapPoint & pt,
            boost::make_iterator_range(poly.vertices_begin(), poly.vertices_end()))
        {
          polysOut << to_double(pt.x()) << " " << to_double(pt.y()) << "\n";
        }
        polysOut << "\n\n";
        plotCommand << "\"" << polysOutStr << "\" u 1:2 i " << polyIdx
            << " w filledcurves";

        if(face.data().label)
          plotCommand << " lt " << *face.data().label;

        plotCommand << ", \\\n";
        ++polyIdx;
      }
    }

    pltOut << "set size square\n";
    pltOut << "unset key\n";
    pltOut << "set style fill transparent solid 0.3 noborder\n";

    plotEdges(arrangement, &edgesOut);
    plotCommand << "\"" << edgesOutStr
        << "\" u 1:2:3:4 w vectors nohead lt rgb \"#000000\"\n";
    pltOut << plotCommand.str();

    if(polysOut.is_open())
      polysOut.close();
    if(pltOut.is_open())
      pltOut.close();
    if(edgesOut.is_open())
      edgesOut.close();

    return true;
  }

template< typename Map>
  bool
  GnuplotAnchorArrangementPlotter< Map>::outputArrangement(
      const Arrangement & map, const InfoMap & labelInfo) const
  {
    return outputArrangement(map);
  }

template< typename Map>
  void
  GnuplotAnchorArrangementPlotter< Map>::plotEdges(const Arrangement & arr,
      std::ostream * const os) const
  {

    arma::vec2 source, dr;
    for(typename Arrangement::Edge_const_iterator edge = arr.edges_begin(),
        edgesEnd = arr.edges_end(); edge != edgesEnd; ++edge)
    {
      if(!edge->is_fictitious())
      {
        source = math::cgalToArma(edge->source()->point());
        dr = math::cgalToArma(edge->target()->point()) - source;
        *os << source(0) << " " << source(1) << " " << dr(0) << " " << dr(1)
            << "\n";
      }
    }
  }

template< typename Map>
  std::string
  GnuplotAnchorArrangementPlotter< Map>::drawLabel(const double x,
      const double y, const std::string & label) const
  {
    std::stringstream ss;
    ss << "set label \"" << label << "\" at " << x << ", " << y << " centre\n";
    return ss.str();
  }

template< typename Map>
  typename GnuplotAnchorArrangementPlotter< Map>::FacePolygon
  GnuplotAnchorArrangementPlotter< Map>::getFacePolygon(
      const typename Arrangement::Face & face) const
  {
    FacePolygon poly;

    if(face.has_outer_ccb())
    {
      const typename Arrangement::Ccb_halfedge_const_circulator start =
          face.outer_ccb();
      typename Arrangement::Ccb_halfedge_const_circulator cl = start;
      do
      {
        poly.push_back(cl->source()->point());
        ++cl;
      } while(cl != start);
    }

    return poly;
  }

}
}

#endif /* GNUPLOT_ANCHOR_ARRANGEMENT_PLOTTER_DETAIL_H */
