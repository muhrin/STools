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
#include <map>
#include <vector>
#include <utility>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/tokenizer.hpp>
#include <boost/range/iterator_range.hpp>

#include <armadillo>

// includes for defining the Voronoi diagram adaptor
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>

#include <spl/analysis/VoronoiEdgeTracer.h>
#include <spl/utility/Range.h>

// FORWARD DECLARES //////////////////////////
class DataRow;

// TYPEDEFS ////////////////////////////////////
typedef unsigned int InfoType;

typedef ::spl::analysis::VoronoiEdgeTracer<InfoType> EdgeTracer;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

// Arrangements stuff
typedef CGAL::Arr_segment_traits_2< K> ArrangementTraits;
typedef CGAL::Arrangement_2< ArrangementTraits> Arrangement;
typedef ArrangementTraits::Segment_2 ArrSegment;

// typedefs for defining the adaptor
typedef CGAL::Triangulation_vertex_base_with_info_2< InfoType, K> Vb;
typedef CGAL::Triangulation_data_structure_2< Vb> Tds;
typedef CGAL::Delaunay_triangulation_2< K, Tds> Delaunay;
typedef CGAL::Delaunay_triangulation_adaptation_traits_2< Delaunay> AT;
typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<
    Delaunay> AP;
typedef CGAL::Voronoi_diagram_2< Delaunay, AT, AP> VD;
typedef Delaunay::Point Point;

// NAMESPACES ////////////////////////////////
namespace fs = ::boost::filesystem;

// CONSTANTS ////////////////////////////////
static const double PI = ::std::acos(-1.0);

// CLASSES //////////////////////////////////
struct InputOptions
{
  ::std::string inputFile;
  int numSteps;
  bool dontSplitSharedVertices;
};

class AnchorPointArrangement;

class AnchorPoint
{
  typedef ::std::set< AnchorPoint *> Neighbours;
public:
  typedef Neighbours::const_iterator NeighbourIterator;

  AnchorPoint(const size_t idx, const ::arma::vec2 & pos,
      const double maxDisplacement);

  const ::arma::vec2 &
  getAnchorPos() const;
  const ::arma::vec2 &
  getPos() const;
  void
  setPos(const ::arma::vec2 & newPos);

  NeighbourIterator
  neighboursBegin() const;
  NeighbourIterator
  neighboursEnd() const;

  double
  getMaxDisplacement() const;

  size_t
  idx() const;

  size_t
  numNeighbours() const;

  void
  addNeighbour(AnchorPoint * const neighbour);

  bool
  hasNeighbour(AnchorPoint * const neighbour) const;

private:
  void
  swapNeighbours(AnchorPoint * const old, AnchorPoint * const newNeighbour);
  void
  clearNeighbours();

  const size_t idx_;
  const ::arma::vec2 anchorPos_;
  ::arma::vec2 pos_;
  const double maxDisplacementSq_;
  Neighbours neighbours_;

  friend class AnchorPointArrangement;
};

AnchorPoint::AnchorPoint(const size_t idx, const ::arma::vec2 & pos,
    const double maxDisplacement) :
    idx_(idx), anchorPos_(pos), pos_(pos), maxDisplacementSq_(
        maxDisplacement * maxDisplacement)
{
}

const ::arma::vec2 &
AnchorPoint::getAnchorPos() const
{
  return anchorPos_;
}

const ::arma::vec2 &
AnchorPoint::getPos() const
{
  return pos_;
}

void
AnchorPoint::setPos(const ::arma::vec2 & newPos)
{
  if(neighbours_.size() > 2)
    return;

  ::arma::vec2 dr = newPos - anchorPos_;
  const double lenSq = ::arma::dot(dr, dr);
  if(lenSq > maxDisplacementSq_)
    pos_ = anchorPos_ + dr * ::std::sqrt(maxDisplacementSq_ / lenSq);
  else
    pos_ = newPos;
}

AnchorPoint::NeighbourIterator
AnchorPoint::neighboursBegin() const
{
  return neighbours_.begin();
}

AnchorPoint::NeighbourIterator
AnchorPoint::neighboursEnd() const
{
  return neighbours_.end();
}

double
AnchorPoint::getMaxDisplacement() const
{
  return ::std::sqrt(maxDisplacementSq_);
}

size_t
AnchorPoint::numNeighbours() const
{
  return neighbours_.size();
}

size_t
AnchorPoint::idx() const
{
  return idx_;
}

void
AnchorPoint::addNeighbour(AnchorPoint * const neighbour)
{
  neighbours_.insert(neighbour);
}

bool
AnchorPoint::hasNeighbour(AnchorPoint * const neighbour) const
{
  return neighbours_.find(neighbour) != neighbours_.end();
}

void
AnchorPoint::swapNeighbours(AnchorPoint * const old,
    AnchorPoint * const newNeighbour)
{
  const AnchorPoint::Neighbours::iterator it = ::std::find(neighbours_.begin(),
      neighbours_.end(), old);
  // todo: assert(it != neighbours_.end())
  neighbours_.erase(it);
  neighbours_.insert(newNeighbour);
}

void
AnchorPoint::clearNeighbours()
{
  neighbours_.clear();
}

class AnchorPointArrangement
{
  typedef ::std::multimap< const Arrangement::Vertex *, AnchorPoint *> PointsMap;
  typedef ::boost::ptr_vector< AnchorPoint> Points;
public:
  typedef Points::iterator PointsIterator;

  AnchorPointArrangement(const EdgeTracer & edgeTracer,
      const bool splitSharedVertices = true);

  PointsIterator
  beginPoints();
  PointsIterator
  endPoints();
  size_t
  numPoints() const;
  AnchorPoint *
  getPoint(const size_t index);

  ::arma::mat
  getPointPositions() const;
  void
  setPointPositions(const ::arma::mat & pos);

private:
  AnchorPoint *
  getAnchorPoint(const Arrangement::Vertex_const_handle & vertex);
  Arrangement::Halfedge_around_vertex_const_circulator
  getFirstZoneCirculator(const Arrangement::Vertex & vertex) const;
  AnchorPoint *
  getNeighbouringAnchor(AnchorPoint * const anchor,
      const Arrangement::Vertex_const_handle & neighbourVertex);

  const EdgeTracer & edgeMapper_;
  Points points_;
  PointsMap pointsMap_;
  const bool splitSharedVertices_;
};

AnchorPointArrangement::AnchorPointArrangement(const EdgeTracer & edgeTracer,
    const bool splitSharedVertices) :
    edgeMapper_(edgeTracer), splitSharedVertices_(splitSharedVertices)
{
  using ::std::pair;
  using ::std::make_pair;

  typedef ::std::map< size_t, ::std::pair< size_t, InfoType> > ZoneConnections;
  typedef ::std::vector<
      pair< Arrangement::Halfedge_around_vertex_const_circulator, InfoType> > SurroundingZones;

  static const double MAX_DISPLACEMENT = 0.06;

  const Arrangement & arrangement = edgeMapper_.getArrangement();

  ::arma::vec2 pt;
  for(Arrangement::Vertex_const_iterator it = arrangement.vertices_begin(),
      end = arrangement.vertices_end(); it != end; ++it)
  {
    pt(0) = it->point()[0];
    pt(1) = it->point()[1];
    // Need to store the pointer to help ADL
    const Arrangement::Vertex * const vtx = it.ptr();
    points_.push_back(new AnchorPoint(points_.size(), pt, /*TODO: CALC MIN DIST TO VTX*/
    MAX_DISPLACEMENT));
    pointsMap_.insert(make_pair(vtx, &points_.back()));
  }

  for(PointsMap::iterator it = pointsMap_.begin(), end = pointsMap_.end();
      it != end; ++it)
  {
    if(it->second->numNeighbours() == it->first->degree())
      continue;

    Arrangement::Halfedge_around_vertex_const_circulator cl =
        it->first->incident_halfedges();
    const Arrangement::Halfedge_around_vertex_const_circulator first = cl;
    do
    {
      AnchorPoint * const neighbour = getAnchorPoint(cl->source());
      it->second->addNeighbour(neighbour);
      neighbour->addNeighbour(it->second);
      ++cl;
    }
    while(cl != first);
  }

  if(splitSharedVertices_)
  {
    for(PointsMap::iterator it = pointsMap_.begin(); it != pointsMap_.end();
        /* increment in loop body */)
    {
      const size_t degree = it->second->numNeighbours();

      // Can't split vertices with less than 4 zones
      if(degree < 4)
      {
        ++it;
        continue;
      }

      const Arrangement::Halfedge_around_vertex_const_circulator first =
          it->first->incident_halfedges();
      Arrangement::Halfedge_around_vertex_const_circulator cl = first;
      SurroundingZones zones;
      do
      {
        zones.push_back(make_pair(cl, edgeMapper_.getInfo(cl)));
        ++cl;
      }
      while(cl != first);

      ZoneConnections connections;
      for(size_t i = 0; i < zones.size(); ++i)
      {
        for(size_t j = i + 1; j < zones.size(); ++j)
        {
          if(zones[i].second == zones[j].second)
            connections[i] = ::std::make_pair(j, zones[i].second);
        }
      }
      if(connections.empty())
      {
        ++it;
        continue;
      }

      ::arma::Mat< int> adjacencyMatrix(degree, degree), labelMatrix(degree, degree);
      adjacencyMatrix.zeros();
      labelMatrix.fill(-1);
      // Perform collision detection
      BOOST_FOREACH(ZoneConnections::const_reference conn, connections)
      {
        for(size_t i = conn.first; i <= conn.second.first; ++i)
        {
          // Sweep left to right through the matrix
          if(labelMatrix(conn.first, i) == -1)
          {
            labelMatrix(conn.first, i) = conn.second.second;
            adjacencyMatrix(conn.first, i) += 1;
          }
          else if(labelMatrix(conn.first, i) != static_cast<int>(conn.second.second))
            adjacencyMatrix(conn.first, i) += 1;

          // Sweep top to bottom through the matrix
          if(labelMatrix(i, conn.second.first) == -1)
          {
            labelMatrix(i, conn.second.first) = conn.second.second;
            adjacencyMatrix(i, conn.second.first) += 1;
          }
          else if(labelMatrix(i, conn.second.first) != static_cast<int>(conn.second.second))
            adjacencyMatrix(i, conn.second.first) += 1;
        }
      }

      { // Get rid of colliding edges
        ZoneConnections::iterator it;
        for(size_t i = 0; i < degree; ++i)
        {
          for(size_t j = i + 1; j < degree; ++j)
          {
            if(adjacencyMatrix(i, j) > 1)
            {
              it = connections.find(i);
              if(it != connections.end())
                connections.erase(it);
              it = connections.find(j);
              if(it != connections.end())
                connections.erase(it);
            }
          }
        }
      }

      ZoneConnections::const_iterator connectIt;
      size_t startZone;
      for(startZone = 0; startZone < degree; ++startZone)
      {
        connectIt = connections.find(startZone);
        if(connectIt != connections.end())
          break;
      }

      AnchorPoint * const originalAnchor = it->second;
      AnchorPoint * currentAnchor = originalAnchor;
      currentAnchor->clearNeighbours();

      ::std::vector< bool> zoneDone(degree, false);
      for(size_t k = 0; k < degree; ++k)
      {
        const size_t i = (k + startZone) % degree; // Wrap around the index

        const Arrangement::Vertex_const_handle neighbour = zones[i].first->source();

        // Check if the current zone already has an anchor point
        if(!zoneDone[i])
        {
          AnchorPoint * const neighbourAnchor = getNeighbouringAnchor(
              originalAnchor, neighbour);
          currentAnchor->addNeighbour(neighbourAnchor);
          if(originalAnchor != currentAnchor)
            neighbourAnchor->swapNeighbours(originalAnchor, currentAnchor);
        }

        connectIt = connections.find(i);
        if(connectIt != connections.end())
        {
          const size_t connectedZoneIdx = (connectIt->second.first + 1) % degree;
          Arrangement::Vertex_const_handle connectedNeighbour =
              zones[connectedZoneIdx].first->source();

          AnchorPoint * const connectedAnchor = getNeighbouringAnchor(
              originalAnchor, connectedNeighbour);
          currentAnchor->addNeighbour(connectedAnchor);
          if(originalAnchor != currentAnchor)
            connectedAnchor->swapNeighbours(originalAnchor, currentAnchor);

          zoneDone[connectedZoneIdx] = true;

          // Create the next anchor
          points_.push_back(
              new AnchorPoint(points_.size(), originalAnchor->getPos(),
                  MAX_DISPLACEMENT));
          currentAnchor = &points_.back();
          pointsMap_.insert(make_pair(it->first, currentAnchor));
        }
        zoneDone[i] = true;
      }
    }
  }
}

AnchorPointArrangement::PointsIterator
AnchorPointArrangement::beginPoints()
{
  return points_.begin();
}

AnchorPointArrangement::PointsIterator
AnchorPointArrangement::endPoints()
{
  return points_.end();
}

size_t
AnchorPointArrangement::numPoints() const
{
  return points_.size();
}

AnchorPoint *
AnchorPointArrangement::getPoint(const size_t index)
{
  return &points_[index];
}

::arma::mat
AnchorPointArrangement::getPointPositions() const
{
  ::arma::mat pos(2, numPoints());
  for(size_t i = 0; i < numPoints(); ++i)
    pos.col(i) = points_[i].getPos();
  return pos;
}

void
AnchorPointArrangement::setPointPositions(const ::arma::mat & pos)
{
  for(size_t i = 0; i < numPoints(); ++i)
    points_[i].setPos(pos.col(i));
}

AnchorPoint *
AnchorPointArrangement::getAnchorPoint(
    const Arrangement::Vertex_const_handle & vertex)
{
  const PointsMap::iterator it = pointsMap_.find(vertex.ptr());
  return it->second;
}

Arrangement::Halfedge_around_vertex_const_circulator
AnchorPointArrangement::getFirstZoneCirculator(
    const Arrangement::Vertex & vertex) const
{
  const Arrangement::Halfedge_around_vertex_const_circulator first =
      vertex.incident_halfedges();
  const InfoType firstInfo = edgeMapper_.getInfo(first);

  Arrangement::Halfedge_around_vertex_const_circulator cl = first;
  ++cl;
  if(edgeMapper_.getInfo(cl) == firstInfo)
    cl = first; // The first one we got was the right one
  else
  {
    cl = first;
    --cl; // One before that
  }

  return cl;
}

AnchorPoint *
AnchorPointArrangement::getNeighbouringAnchor(AnchorPoint * const anchor,
    const Arrangement::Vertex_const_handle & neighbourVertex)
{
  const ::std::pair< PointsMap::iterator, PointsMap::iterator> range =
      pointsMap_.equal_range(neighbourVertex.ptr());

  BOOST_FOREACH(PointsMap::reference entry, range)
  {
    if(entry.second->hasNeighbour(anchor))
      return entry.second;
  }
  return NULL;
}

// TYPEDEFS /////////////////////////////////
typedef ::std::vector< ::std::pair< Point, InfoType> > Points;

// CONSTANTS /////////////////////////////////

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

  Delaunay tempDelaunay;
  tempDelaunay.insert(points.begin(), points.end());
  VD voronoi(tempDelaunay, true);

  EdgeTracer tracer(voronoi);

  AnchorPointArrangement arr(tracer, !in.dontSplitSharedVertices);

  ::arma::mat forces(2, arr.numPoints()), forcesDot(2, arr.numPoints()), pos = arr.getPointPositions();
  ::arma::rowvec forcesSq(arr.numPoints());
  ::arma::vec2 r1, r2, r1Perp, r2Perp, f1, f2;
  AnchorPoint * n1, *n2;
  double len1, len2, theta, torque;
  const double kappa = 1.0, stepsize = 1e-4;
  for(int iter = 0; iter < in.numSteps; ++iter)
  {
    forces.zeros();
    BOOST_FOREACH(AnchorPoint & point,
        ::boost::make_iterator_range(arr.beginPoints(), arr.endPoints()))
    {
      if(point.numNeighbours() > 2)
        continue;

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
          torque = -kappa * (theta - PI);

          f1 = k * (torque / len1) * r1Perp;
          f2 = k * (torque / len2) * r2Perp;
//          if(point.numNeighbours() < 3)
//          {
            forces.col(n1->idx()) += f1;
            forces.col(n2->idx()) -= f2;
//          }
          forces.col(point.idx()) += f2 - f1;
        }
      }
    }

    pos += stepsize * forces;
    arr.setPointPositions(pos);
    pos = arr.getPointPositions();

    forcesDot = forces % forces;
    forcesSq = ::arma::sum(forcesDot);
    if(forcesSq.max() < 0.00001)
      break;
  }

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
        ("steps,n", po::value< int>(&in.numSteps)->default_value(100), "number of smoothing steps")
        ("no-split-shared,s", po::value<bool>(&in.dontSplitSharedVertices)->default_value(false)->zero_tokens(), "do not split shared vertices");

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
