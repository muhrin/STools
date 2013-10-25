/*
 * smap2.cpp
 *
 *  Created on: Sep 23, 2013
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "STools.h"

#include <spl/SSLib.h>
#ifdef SSLIB_USE_CGAL

#include <algorithm>
#include <fstream>
#include <limits>
#include <map>
#include <vector>
#include <utility>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <boost/range/iterator_range.hpp>

#include <armadillo>

#include <CGAL/Polygon_2.h>

#include <spl/analysis/AnchorPointArrangement.h>
#include <spl/analysis/VoronoiEdgeTracer.h>
#include <spl/utility/Range.h>

// FORWARD DECLARES //////////////////////////
class DataRow;

// TYPEDEFS ////////////////////////////////////
typedef unsigned int InfoType;
typedef ::spl::analysis::AnchorPointArrangement<InfoType> AnchorPointArrangement;
typedef AnchorPointArrangement::Arrangement Arrangement;
typedef AnchorPointArrangement::EdgeTracer EdgeTracer;
typedef EdgeTracer::Point Point;
typedef ::std::vector< ::std::pair< Point, InfoType> > Points;

// NAMESPACES ////////////////////////////////
namespace fs = ::boost::filesystem;

// CONSTANTS ////////////////////////////////
static const double PI = ::std::acos(-1.0);

// CLASSES //////////////////////////////////
struct InputOptions
{
  ::std::string inputFile;
  int numSteps;
  double forceTolerance;
  bool dontSplitSharedVertices;
  double kappa;
  double vertexForceStrength;
  double areaForceStrength;
};


// CONSTANTS /////////////////////////////////
static const double FORCE_TOL_DEFAULT = 1e-3;

// FUNCTIONS ////////////////////////////////
template< typename T>
  double
  sgn(T val)
  {
    return (T(0) < val) - (val < T(0));
  }

int
processCommandLineArgs(InputOptions & in, const int argc, char * argv[]);

int
main(const int argc, char * argv[])
{
  typedef boost::tokenizer< boost::char_separator< char> > Tok;

  using ::boost::lexical_cast;
  using ::spl::analysis::AnchorPoint;

  static const boost::char_separator< char> tokSep(" \t");

  feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);

  // Program options
  InputOptions in;

  int result = processCommandLineArgs(in, argc, argv);
  if(result != 0)
    return result;

  if(!fs::exists(fs::path(in.inputFile)))
  {
    ::std::cerr << "Input file " << in.inputFile << " does not exist.";
    return 1;
  }

  Points points;
  ::std::ifstream inFile(in.inputFile.c_str());
  if(inFile.is_open())
  {
    ::std::string line;
    while(::std::getline(inFile, line))
    {
      if(!line.empty() && line[0] != '#')
      {
        Tok toker(line, tokSep);
        Tok::iterator it = toker.begin();
        if(it == toker.end())
          continue;

        double x, y;
        unsigned int label;

        bool foundAll = false;
        try
        {
          x = lexical_cast< double>(*it);
          if(++it == toker.end())
            continue;

          y = ::boost::lexical_cast< double>(*it);
          if(++it == toker.end())
            continue;

          label = ::boost::lexical_cast< int>(*it);
          foundAll = true;
        }
        catch(const ::boost::bad_lexical_cast & /*e*/)
        {
        }
        if(foundAll)
          points.push_back(::std::make_pair(Point(x, y), label));
      }
    }

    inFile.close();
  }

  EdgeTracer::Delaunay tempDelaunay;
  tempDelaunay.insert(points.begin(), points.end());
  EdgeTracer::Voronoi voronoi(tempDelaunay, true);

  EdgeTracer tracer(voronoi);

  AnchorPointArrangement arr(tracer);

  typedef AnchorPointArrangement::Arrangement::Face Face;
  typedef ::std::map<const Face *, double> FaceAreas;

  const AnchorPointArrangement::Arrangement & arrangement = tracer.getArrangement();
  FaceAreas faces;
  BOOST_FOREACH(const Face & face,
      ::boost::make_iterator_range(arrangement.faces_begin(), arrangement.faces_end()))
  {
    const ::boost::optional<double> area = arr.getFaceAnchorArea(face);
    if(area)
      faces[&face] = *area;
  }

  ::arma::mat forces(2, arr.numPoints()), forcesDot(2, arr.numPoints()),
      pos = arr.getPointPositions();
  ::arma::rowvec forcesSq(arr.numPoints());
  ::arma::vec2 r1, r2, r1Perp, r2Perp, f1, f2;
  AnchorPoint * n1, *n2;
  double len1, len2, lenSq, theta, torque, forceSqMax;
  const double stepsize = 1e-4;
  int iter;
  const int maxIter = in.numSteps == -1 ? ::std::numeric_limits<int>::max() : in.numSteps;
  for(iter = 0; iter < maxIter; ++iter)
  {
    forces.zeros();
    BOOST_FOREACH(AnchorPoint & point,
        ::boost::make_iterator_range(arr.beginPoints(), arr.endPoints()))
    {
      if(point.numNeighbours() > 2)
      {
        // Move the connecting vertex to the midpoint of the vertices
        // that connect to it
        r1.zeros();
        // Loop over neighbours
        for(AnchorPoint::NeighbourIterator n1It = point.neighboursBegin(), end =
            point.neighboursEnd(); n1It != end; ++n1It)
        {
          r1 += pos.col((*n1It)->idx());
        }
        f1 = r1 / static_cast<float>(point.numNeighbours()) - pos.col(point.idx());
        forces.col(point.idx()) += in.vertexForceStrength * f1;
      }
      else
      {

        // Loop over neighbours pairs
        for(AnchorPoint::NeighbourIterator n1It = point.neighboursBegin(), end =
            point.neighboursEnd(); n1It != end; ++n1It)
        {
          n1 = *n1It;
          r1 = pos.col(n1->idx()) - pos.col(point.idx());
          len1 = ::std::sqrt(::arma::dot(r1, r1));
          r1Perp(0) = r1(1) / len1;
          r1Perp(1) = -r1(0) / len1;

          for(AnchorPoint::NeighbourIterator n2It =
              ++AnchorPoint::NeighbourIterator(n1It); n2It != end; ++n2It)
          {

            n2 = *n2It;
            r2 = pos.col(n2->idx()) - pos.col(point.idx());
            len2 = ::std::sqrt(::arma::dot(r2, r2));
            r2Perp(0) = r2(1) / len2;
            r2Perp(1) = -r2(0) / len2;

            // Have to do this check so acos doesn't get a number that is ever
            // so slightly out of the allowed range [-1:1]
            double dp = ::arma::dot(r1, r2) / (len1 * len2);
            dp = ::std::min(dp, 1.0);
            dp = ::std::max(dp, -1.0);

            // Use the sign from the cross product
            const double k = sgn(r1(0) * r2(1) - (r1(1) * (r2(0))));

            theta = ::std::acos(dp);
            torque = -in.kappa * (theta - PI);

            f1.zeros();
            f2.zeros();

            f1 = k * (torque / len1) * r1Perp;
            f2 = k * (torque / len2) * r2Perp;

            forces.col(n1->idx()) += f1;
            forces.col(n2->idx()) -= f2;

            forces.col(point.idx()) += f2 - f1;
          }
        }
      }
    }

    BOOST_FOREACH(FaceAreas::reference face, faces)
    {
      const double areaDiff = (*arr.getFaceAnchorArea(*face.first) - face.second) / face.second;
      //const double areaDiff = *arr.getFaceAnchorArea(*face.first) / face.second - 1.0;

      const Arrangement::Ccb_halfedge_const_circulator first = face.first->outer_ccb();
      Arrangement::Ccb_halfedge_const_circulator edge1 = first, edge2 = first;
      const AnchorPoint * anchor;
      do
      {
        ++edge2;

        // The halfedges supplied go around the boundary in the counterclockwise
        // direction
        n1 = edge1->source()->data().anchor;
        anchor = edge1->target()->data().anchor;
        n2 = edge2->target()->data().anchor;
        r1 = pos.col(n2->idx()) - pos.col(n1->idx());
        len1 = ::std::sqrt(::arma::dot(r1, r1));
        // This gives a vector pointing OUT of the face
        r1Perp(0) = r1(1) / len1;
        r1Perp(1) = -r1(0) / len1;
        forces.col(anchor->idx()) -= in.areaForceStrength * areaDiff * r1Perp;

        edge1 = edge2;
      } while(edge1 != first);
    }

    pos += stepsize * forces;

    // Add forces to keep the points no further than the maximum displacement
    // distance away from the anchor position and calculate the force required to
    // do so
    BOOST_FOREACH(AnchorPoint & point,
        ::boost::make_iterator_range(arr.beginPoints(), arr.endPoints()))
    {
      r1 = pos.col(point.idx()) - point.getAnchorPos();
      lenSq = ::arma::dot(r1, r1);
      if(lenSq > 0.0 && lenSq > (point.getMaxDisplacement() * point.getMaxDisplacement()))
      {
        len1 = ::std::sqrt(lenSq);
        const ::arma::vec2 dr = r1 * (point.getMaxDisplacement() / len1) - r1;
        pos.col(point.idx()) += dr;
        forces.col(point.idx()) += dr / stepsize;
      }
    }

    forcesDot = forces % forces;
    forcesSq = ::arma::sum(forcesDot);
    forceSqMax = forcesSq.max();
    if(forceSqMax < in.forceTolerance)
      break;

    arr.setPointPositions(pos);
  }
  arr.setPointPositions(pos);

  ::arma::vec2 vR, vDr;
  for(AnchorPointArrangement::PointsIterator it = arr.beginPoints(), end =
      arr.endPoints(); it != end; ++it)
  {
    vR = it->getPos();
    for(AnchorPoint::NeighbourIterator nIt = it->neighboursBegin(), nEnd =
        it->neighboursEnd(); nIt != nEnd; ++nIt)
    {
      vDr = (*nIt)->getPos() - vR;
      ::std::cout << vR(0) << " " << vR(1) << " " << vDr(0) << " " << vDr(1)
          << "\n";
    }
  }

  return 0;
}

int
processCommandLineArgs(InputOptions & in, const int argc, char * argv[])
{
  namespace po = ::boost::program_options;

  const ::std::string exeName(argv[0]);

  try
  {
    po::options_description general(
        "smap\nUsage: " + exeName + " [options] input_file...\nOptions");
    general.add_options()
        ("help", "Show help message")
        ("input-file", po::value< ::std::string>(&in.inputFile), "input file")
        ("steps,n", po::value< int>(&in.numSteps)->default_value(-1), "number of smoothing steps (-1 = until tolerance is satisfied)")
        ("force-tol,f", po::value< double>(&in.forceTolerance)->default_value(FORCE_TOL_DEFAULT), "force tolerance used as stopping criterion")
        ("no-split-shared,s", po::value<bool>(&in.dontSplitSharedVertices)->default_value(false)->zero_tokens(), "do not split shared vertices")
        ("kappa,k", po::value<double>(&in.kappa)->default_value(1.0), "torque spring force strength, used to smooth edges")
        ("area-force,a", po::value<double>(&in.areaForceStrength)->default_value(100.0), "area force strength, used to keep maintain the area of plot regions")
        ("vertex-force,v", po::value<double>(&in.vertexForceStrength)->default_value(400.0), "vertex force strength, used to keep vertices joining 3 or more edges centered");

    po::positional_options_description p;
    p.add("input-file", 1);

    po::options_description cmdLineOptions;
    cmdLineOptions.add(general);

    po::variables_map vm;
    po::store(
        po::command_line_parser(argc, argv).options(cmdLineOptions).positional(
            p).run(), vm);

    // Deal with help first, otherwise missing required parameters will cause exception on vm.notify
    if(vm.count("help"))
    {
      ::std::cout << cmdLineOptions << ::std::endl;
      return 1;
    }

    po::notify(vm);
  }
  catch(std::exception& e)
  {
    ::std::cout << e.what() << "\n";
    return 1;
  }

  // Everything went fine
  return 0;
}

#endif /* SSLIB_USE_CGAL */
