// TODO(avk): Docstrings missing!

#ifndef LRASTAR_UTILS_ROADMAPFROMFILE_HPP_
#define LRASTAR_UTILS_ROADMAPFROMFILE_HPP_

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <cmath>
#include <stdlib.h>

#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <ompl/base/State.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace LRAstar {
namespace utils {

/* RoadmapFromFilePutStateMap */
/// The map used to decode the .graphml file and populate the vertex states.
/// \tparam PropMap The type of property map for vertex states.
/// \tparam StateWrapper The wrapper for the ompl state.
template <class PropMap, class StateWrapper>
class RoadmapFromFilePutStateMap
{
public:
  typedef boost::writable_property_map_tag category;
  typedef typename boost::property_traits<PropMap>::key_type key_type;
  typedef std::string value_type;
  typedef std::string reference;

  const PropMap mPropMap;
  ompl::base::StateSpacePtr mSpace;
  const size_t mDim;

  RoadmapFromFilePutStateMap(PropMap propMap, ompl::base::StateSpacePtr space, size_t dim)
  : mPropMap{propMap}
  , mSpace{space}
  , mDim{dim}
  {
  }
};

// Do not allow calling get on this property map
template <class PropMap, class StateWrapper>
inline std::string
get(const RoadmapFromFilePutStateMap<PropMap,StateWrapper> &map,
  const typename RoadmapFromFilePutStateMap<PropMap,StateWrapper>::key_type &k)
{
  abort();
}

/// Convert string representation of vector state space to ompl state
template <class PropMap, class StateWrapper>
inline void
put(const RoadmapFromFilePutStateMap<PropMap,StateWrapper> &map,
  const typename RoadmapFromFilePutStateMap<PropMap,StateWrapper>::key_type &k,
  const std::string representation)
{
  get(map.mPropMap, k).reset(new StateWrapper(map.mSpace));
  ompl::base::State *ver_state{get(map.mPropMap, k)->state};
  double *values{ver_state->as<ompl::base::RealVectorStateSpace::StateType>()->values};
  std::stringstream ss(representation);
  for (size_t ui = 0; ui < map.mDim; ui++)
  {
    ss >> values[ui];
  }
}

/* RoadmapFromFilePutEdgeLengthMap */
/// The map used to decode the .graphml file and populate the edge length
/// \tparam PropMap The type of property map for vertex states
template <class PropMap>
class RoadmapFromFilePutEdgeLengthMap
{
public:
  typedef boost::writable_property_map_tag category;
  typedef typename boost::property_traits<PropMap>::key_type key_type;
  typedef std::string value_type;
  typedef std::string reference;
  const PropMap mPropMap;

  RoadmapFromFilePutEdgeLengthMap(PropMap propMap):
    mPropMap(propMap)
  {
  }
};

/// Do not allow calling get on this property map
template <class PropMap>
inline std::string
get(const RoadmapFromFilePutEdgeLengthMap<PropMap> &map,
  const typename RoadmapFromFilePutEdgeLengthMap<PropMap>::key_type &k)
{
  abort();
}

template <class PropMap>
inline void
put(const RoadmapFromFilePutEdgeLengthMap<PropMap> &map,
  const typename RoadmapFromFilePutEdgeLengthMap<PropMap>::key_type &k,
  const std::string representation)
{
  put(map.mPropMap, k, stod(representation));
}

/* RoadmapFromFilePutEdgePriorMap */
/// The map used to decode the .graphml file and populate the edge priors
/// \tparam PropMap The type of property map for edge prior
template <class PropMap>
class RoadmapFromFilePutEdgePriorMap
{
public:
  typedef boost::writable_property_map_tag category;
  typedef typename boost::property_traits<PropMap>::key_type key_type;
  typedef std::string value_type;
  typedef std::string reference;
  const PropMap mPropMap;

  RoadmapFromFilePutEdgePriorMap(PropMap propMap):
    mPropMap(propMap)
  {
  }
};

/// Do not allow calling get on this property map
template <class PropMap>
inline std::string
get(const RoadmapFromFilePutEdgePriorMap<PropMap> &map,
  const typename RoadmapFromFilePutEdgePriorMap<PropMap>::key_type &k)
{
  abort();
}

template <class PropMap>
inline void
put(const RoadmapFromFilePutEdgePriorMap<PropMap> &map,
  const typename RoadmapFromFilePutEdgePriorMap<PropMap>::key_type &k,
  const std::string representation)
{
  put(map.mPropMap, k, stod(representation));
}


/* RoadmapFromFile */
template <class Graph, class VStateMap, class StateWrapper, class ELength, class EPrior>
class RoadmapFromFile
{
  typedef boost::graph_traits<Graph> GraphTypes;
  typedef typename GraphTypes::vertex_descriptor Vertex;
  typedef typename GraphTypes::vertex_iterator VertexIter;
  typedef typename GraphTypes::edge_descriptor Edge;
  typedef typename GraphTypes::edge_iterator EdgeIter;

  public:
    const std::string mFilename;

    RoadmapFromFile(
      const ompl::base::StateSpacePtr space,
      std::string filename)
    : mSpace(space)
    , mFilename(filename)
    , mBounds(0)
    {
      if (mSpace->getType() != ompl::base::STATE_SPACE_REAL_VECTOR)
        throw std::runtime_error("This only supports real vector state spaces!");
      
      mDim = mSpace->getDimension();
      mBounds = mSpace->as<ompl::base::RealVectorStateSpace>()->getBounds();
    }

    ~RoadmapFromFile() {}

    void generate(Graph &g, VStateMap stateMap, ELength lengthMap, EPrior priorMap)
    {
      boost::dynamic_properties props;
      props.property("state", RoadmapFromFilePutStateMap<VStateMap, StateWrapper>(stateMap, mSpace, mDim));
      props.property("length", RoadmapFromFilePutEdgeLengthMap<ELength>(lengthMap));
      props.property("prior", RoadmapFromFilePutEdgePriorMap<EPrior>(priorMap));

      std::ifstream fp;
      fp.open(mFilename.c_str());
      boost::read_graphml(fp, g, props);
      fp.close();

      EdgeIter ei, ei_end;
      for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
      {
        ompl::base::State *state1 = get(stateMap, source(*ei, g))->state;
        ompl::base::State *state2 = get(stateMap, target(*ei, g))->state;
        put(lengthMap, *ei, mSpace->distance(state1, state2));
      }
    }

  private:
    size_t mDim;
    ompl::base::RealVectorBounds mBounds;
    const ompl::base::StateSpacePtr mSpace;
};

} // namespace utils
} // namespace LRAstar

#endif // LRASTAR_UTILS_ROADMAPFROMFILE_HPP_
