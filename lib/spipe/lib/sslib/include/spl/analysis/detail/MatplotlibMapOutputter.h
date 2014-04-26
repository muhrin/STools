/*
 * MatplotLibMapOutputter.h
 *
 *  Created on: Nov 11, 2013
 *      Author: Martin Uhrin
 */

#ifndef MATPLOTLIB_MAP_OUTPUTTER_DETAIL_H
#define MATPLOTLIB_MAP_OUTPUTTER_DETAIL_H

// INCLUDES ///////////////////
#include <boost/foreach.hpp>

// FORWARD DECLARATIONS ///////

// DEFINITION ///////////////////////

namespace spl {
namespace analysis {

template< typename MapTraits>
  class MatplotlibMapOutputter< MapTraits>::FacePath
  {
    typedef typename Arrangement::Point_2 Point;
    typedef typename Arrangement::X_monotone_curve_2 Curve;
  public:
    explicit
    FacePath(std::ostream * const os) :
        myPathStarted(false), myOs(os)
    {
      *myOs << "face_data = [\n";
    }
    ~FacePath()
    {
      *myOs << INDENT << "]\n";
      *myOs << "face_codes, face_verts = zip(*face_data)\n";
      *myOs << "face_path = " << PATH << "(face_verts, face_codes)\n";
      *myOs << "face_patch = " << PATH_PATCH
          << "(face_path, facecolor='r', alpha=0.5)\n";
      *myOs << PLOT << ".add_patch(face_patch)\n";
    }

    void
    halfedge(const typename Arrangement::Halfedge & he)
    {
      const bool reverse = !he.curve().source().is_same(he.source()->point());
      if(!myPathStarted)
      {
        if(reverse)
          moveTo(he.target()->point());
        else
          moveTo(he.source()->point());
        myPathStarted = true;
      }
      curveTo(he.curve(), reverse);
    }

  private:
    void
    moveTo(const Point & p)
    {
      *myOs << INDENT << "(" << PATH << ".MOVETO, " << point(p) << "),\n";
    }
    void
    lineTo(const Point & p)
    {
      *myOs << INDENT << "(" << PATH << ".LINETO, " << point(p) << "),\n";
    }
    void
    curveTo(const Curve & c, const bool reverse)
    {
      const typename MapTraits::ArrTraits::Curve_2 & supportingCurve =
          c.supporting_curve();
      const size_t cPoints = supportingCurve.number_of_control_points();
      if(cPoints == 2)
      {
        if(reverse)
          lineTo(c.source());
        else
          lineTo(c.target());
      }
      else if(cPoints == 4)
      {
        if(reverse)
        {
          for(size_t i = 3; i > 0; --i)
            *myOs << INDENT << "(" << PATH << ".CURVE4, "
                << point(supportingCurve.control_point(i)) << "),\n";
        }
        else
        {
          for(size_t i = 1; i < 4; ++i)
            *myOs << INDENT << "(" << PATH << ".CURVE4, "
                << point(supportingCurve.control_point(i)) << "),\n";
        }
      }
    }
    std::string
    point(const Point & p)
    {
      std::stringstream ss;
      ss << "(" << p.x() << ", " << p.y() << ")";
      return ss.str();
    }
    std::string
    point(const typename MapTraits::ArrTraits::Rat_point_2 & p)
    {
      std::stringstream ss;
      ss << "(" << CGAL::to_double(p.x()) << ", " << CGAL::to_double(p.y())
          << ")";
      return ss.str();
    }

    bool myPathStarted;
    std::ostream * const myOs;
  };

template< typename MapTraits>
  const std::string MatplotlibMapOutputter< MapTraits>::PATH = "mpath.Path";
template< typename MapTraits>
  const std::string MatplotlibMapOutputter< MapTraits>::PATH_PATCH =
      "mpatches.PathPatch";
template< typename MapTraits>
  const std::string MatplotlibMapOutputter< MapTraits>::INDENT = "    ";
template< typename MapTraits>
  const std::string MatplotlibMapOutputter< MapTraits>::PLOT = "ax";

template< typename MapTraits>
  void
  MatplotlibMapOutputter< MapTraits>::outputArrangement(const Arrangement & map,
      std::ostream * const os) const
  {
    out(map, NULL, os);
  }

template< typename MapTraits>
  void
  MatplotlibMapOutputter< MapTraits>::outputArrangement(const Arrangement & map,
      const LabelNames & labelNames, std::ostream * const os) const
  {
    out(map, &labelNames, os);
  }

template< typename MapTraits>
  std::string
  MatplotlibMapOutputter< MapTraits>::fileExtension() const
  {
    return "py";
  }

template< typename MapTraits>
  void
  MatplotlibMapOutputter< MapTraits>::out(const Arrangement & map,
      const LabelNames * const names, std::ostream * const os) const
  {
    std::cout << map.number_of_faces() << std::endl;

    *os << "import matplotlib.path as mpath\n"
        << "import matplotlib.patches as mpatches\n"
        << "import matplotlib.pyplot as plt\n\n"
        << "fig, ax = plt.subplots()\n\n";

    BOOST_FOREACH(const Face & face,
        boost::make_iterator_range(map.faces_begin(), map.faces_end()))
    {
      drawFace(face, os);
      if(names && face.data().label)
      {
        const typename LabelNames::const_iterator it = names->find(
            *face.data().label);
//        drawLabel(it->second, pos, os);
      }
    }

    *os << "\nax.autoscale_view()\n";
    *os << "plt.show()\n";
  }

template< typename MapTraits>
  void
  MatplotlibMapOutputter< MapTraits>::drawFace(const Face & face,
      std::ostream * const os) const
  {
    if(!face.is_unbounded())
    {
      FacePath draw(os);
      const typename Arrangement::Ccb_halfedge_const_circulator first =
          face.outer_ccb();
      typename Arrangement::Ccb_halfedge_const_circulator cl = first;
      do
      {
        draw.halfedge(*cl);
        ++cl;
      } while(cl != first);
    }
  }

}
}

#endif /* MATPLOTLIB_MAP_OUTPUTTER_DETAIL_H */
