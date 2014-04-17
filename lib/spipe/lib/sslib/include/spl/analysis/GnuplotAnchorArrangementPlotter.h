/*
 * GnuplotAnchorArrangementPlotter.h
 *
 *  Created on: Jul 17, 2013
 *      Author: Martin Uhrin
 */

#ifndef GNUPLOT_ANCHOR_ARRANGEMENT_PLOTTER_H
#define GNUPLOT_ANCHOR_ARRANGEMENT_PLOTTER_H

// INCLUDES ////////////
#include "spl/SSLib.h"

#ifdef SPL_WITH_CGAL

#include <ostream>
#include <string>

#include <CGAL/Polygon_2.h>

#include "spl/analysis/AnchorArrangementOutputter.h"

// DEFINITION ///////////////////////

namespace spl {

// FORWARD DECLARATIONS ///////

namespace analysis {

template< typename Map>
  class GnuplotAnchorArrangementPlotter : public AnchorArrangementOutputter< Map>
  {
    typedef typename Map::Arrangement Arrangement;
    typedef CGAL::Polygon_2< typename Map::Kernel> FacePolygon;
  public:
    typedef typename AnchorArrangementOutputter< Map>::InfoMap InfoMap;

    GnuplotAnchorArrangementPlotter(const std::string & outputFileStem);
    virtual
    ~GnuplotAnchorArrangementPlotter()
    {
    }

    virtual bool
    outputArrangement(const Arrangement & map) const;
    virtual bool
    outputArrangement(const Arrangement & map, const InfoMap & labelInfo) const;

  private:
    void
    plotEdges(const Arrangement & arrangement, std::ostream * const os) const;
    std::string
    drawLabel(const double x, const double y, const std::string & label) const;
    FacePolygon
    getFacePolygon(const typename Arrangement::Face & face) const;

    const std::string myOutputFileStem;
  };

}
}

#include "spl/analysis/detail/GnuplotAnchorArrangementPlotter.h"

#endif // SPL_WITH_CGAL
#endif /* GNUPLOT_ANCHOR_ARRANGEMENT_PLOTTER_H */
