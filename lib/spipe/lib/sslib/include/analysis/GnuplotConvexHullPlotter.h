/*
 * GnuplotConvexHullPlotter.h
 *
 *  Created on: Jul 17, 2013
 *      Author: Martin Uhrin
 */

#ifndef GNUPLOT_CONVEX_HULL_PLOTTER_H
#define GNUPLOT_CONVEX_HULL_PLOTTER_H

// INCLUDES ////////////
#include "SSLib.h"

#ifdef SSLIB_USE_CGAL

#include "analysis/ConvexHull.h"
#include "analysis/IConvexHullOutputter.h"

// DEFINITION ///////////////////////

namespace sstbx
{

// FORWARD DECLARATIONS ///////

namespace analysis
{

class GnuplotConvexHullPlotter : public IConvexHullOutputter
{
public:

  GnuplotConvexHullPlotter();

  virtual bool outputHull(const ConvexHull & convexHull) const;
  virtual bool outputHull(const ConvexHull & convexHull, const IConvexHullInfoSupplier & infoSupplier) const;

  bool getDrawTieLines() const;
  void setDrawTieLines(const bool draw);

  bool getSupressEnergyDimension() const;
  void setSupressEnergyDimension(const bool supress);

private:
  ::std::string printVec(const ConvexHull::VectorD & vec) const;
  ::std::string printPoint(const ConvexHull::PointD & point) const;

  void drawBoundary(::std::ostream & os, const ConvexHull & convexHull) const;
  void drawTieLines(::std::ostream & os, const ConvexHull & convexHull) const;

  ::std::string myOutputStem;
  bool myDrawBoundary;
  bool myDrawTieLines;
  bool mySupressEnergyDimension;
};

}
}

#endif // SSLIB_USE_CGAL
#endif /* GNUPLOT_CONVEX_HULL_PLOTTER_H */
