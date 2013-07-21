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
  virtual bool outputHull(const ConvexHull & convexHull, const IConvexHullInfoSupplier * const infoSupplier) const;

  bool getDrawTieLines() const;
  void setDrawTieLines(const bool draw);

  bool getSupressEnergyDimension() const;
  void setSupressEnergyDimension(const bool supress);

  void setDrawHullLabels(const bool label);

  void setDrawOffHullPoints(const bool draw);

private:

  static const double LABEL_MARGIN;

  class Plot
  {
  public:
    Plot();
    ::std::string drawLine(const ConvexHull::PointD & x0, const ConvexHull::PointD & x1);
    ::std::string drawLine(const ConvexHull::PointD & x0, const ConvexHull::PointD & x1, const int lineStyle);
    ::std::string drawLabel(const ::std::string label, const ConvexHull::PointD & x) const;
    ::std::string printPoint(const ConvexHull::PointD & point) const;

  private:
    int myArrowCounter;
    int myLabelCounter;
  };

  ::std::string printPoint(const ConvexHull::PointD & point) const;

  void setStyles(::std::ostream & os, const ConvexHull & convexHull) const;
  void drawBoundary(::std::ostream & os, const ConvexHull & convexHull, Plot & plot) const;
  void drawTieLines(::std::ostream & os, const ConvexHull & convexHull, Plot & plot) const;
  void drawEndpointLabels(::std::ostream & os, const ConvexHull & convexHull, Plot & plot) const;

  int plotDims(const ConvexHull & convexHull) const;

  ConvexHull::PointD prepPoint(const ConvexHull::PointD & point) const;

  ::std::string myOutputStem;
  bool myDrawBoundary;
  bool myDrawTieLines;
  bool myDrawLabels;
  bool myDrawOffHullPoints;
  bool mySupressEnergyDimension;
};

}
}

#endif // SSLIB_USE_CGAL
#endif /* GNUPLOT_CONVEX_HULL_PLOTTER_H */