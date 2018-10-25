/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef LRASTAR_LRASTAR_HPP_
#define LRASTAR_LRASTAR_HPP_

// STL headers
#include <vector>
#include <string> 
#include <unordered_set>
#include <queue>
#include <exception>

// Boost headers
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/property_map/dynamic_property_map.hpp>

// OMPL headers
#include <ompl/base/Planner.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

// LRAstar defined headers
#include "util.hpp"

namespace LRAstar {

/// The OMPL Planner class that implements the algorithm
class LRAstar: public ompl::base::Planner
{
public:
  /// Constructor
  /// \param[in] si The OMPL space information manager
  explicit LRAstar(const ompl::base::SpaceInformationPtr &si);

  /// \param[in] si The OMPL space information manager
  /// \param[in] roadmapFileName The path to the .graphml file that encodes the roadmap.
  /// \param[in] lookahead The lazy lookahead that defines the horizon. Default is 1.
  /// \param[in] greediness The greediness to evaluate lazy shortest paths. Default is 1.
  LRAstar(const ompl::base::SpaceInformationPtr &si,
          const std::string& roadmapFileName,
          double lookahead);

  /// Destructor
  ~LRAstar(void);

  ///////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////

  enum CollisionStatus
  {
    BLOCKED,
    FREE
  };

  ///////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////

  // Properties associated with each roadmap vertex.
  struct VProp
  {
    /// The underlying state of the vertex
    utils::StateWrapperPtr state;

    /// Cost-to-Come
    double costToCome;

    /// Estimate Cost-to-Come
    double lazyCostToCome;

    /// Heuristic value
    double heuristic;

    /// Budget
    double budgetToExtend;

    /// Parent
    std::size_t parent;

    /// Children
    std::vector<std::size_t> children;

    /// Visited
    bool visited;

    /// Collision status
    CollisionStatus status;

  }; // struct VProp

  // Properties associated with each roadmap edge.
  struct EProp
  {
    /// The length of the edge using the space distance metric
    double length;

    /// Flag to check if edge is evaluated
    bool isEvaluated;

    /// States embedded in an edge
    std::vector<utils::StateWrapperPtr> edgeStates;

    /// Collision status
    CollisionStatus status;

    /// Prior over existence of edge
    double prior;

  }; // struct EProp

  ///////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////

  // Helpful alias declarations
  /// Undirected Boost graph
  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VProp, EProp> Graph;

  /// Boost vertex
  typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

  /// Boost vertex iterator
  typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;

  /// Boost edge
  typedef boost::graph_traits<Graph>::edge_descriptor Edge;

  /// Boost edge iterator
  typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;

  /// Boost graph neighbor iterator
  typedef boost::graph_traits<Graph>::adjacency_iterator NeighborIter;

  /// Map each vertex to a unique ID
  typedef boost::property_map<Graph, boost::vertex_index_t VProp::*>::type VertexIndexMap;

  /// Map each vertex to the underlying state [read from the graphml file]
  typedef boost::property_map<Graph, utils::StateWrapperPtr VProp::*>::type VPStateMap;

  /// Map each edge to a unique ID
  typedef boost::property_map<Graph, boost::edge_index_t EProp::*>::type EdgeIndexMap;

  /// Map each edge to its length
  typedef boost::property_map<Graph, double EProp::*>::type EPLengthMap;

  /// Map each edge to its existence prior
  typedef boost::property_map<Graph, double EProp::*>::type EPPriorMap;

  /// Unordered set of graph vertices to track the lazy band.
  struct HashFunction
  {
    std::size_t operator()(const Vertex& v) const
    {
      return v;
    }
  };
  typedef std::unordered_set<Vertex, HashFunction> unorderedSet;

  ///////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////

  // Setters and Getters
  /// Set value of lazy lookahead.
  void setLookahead(double lookahead);

  /// Set roadmap information.
  void setRoadmapFileName(const std::string& roadmapFileName);

  /// Set connection radius
  void setConnectionRadius(double connectionRadius);

  /// Set collision checking radius
  void setCheckRadius(double checkRadius);

  /// Get value of lazy lookahead.
  double getLookahead() const;

  /// Get roadmap information.
  std::string getRoadmapFileName() const;

  /// Get connection radius used to generate the graph.
  double getConnectionRadius() const;

  /// Get resolution of collision checking.
  double getCheckRadius() const;

  /// Get ID of start vertex.
  Vertex getStartVertex() const;

  /// Get ID of goal vertex.
  Vertex getGoalVertex() const;

  /// Get the shortest path cost.
  double getBestPathCost() const;

  // Internal Evaluation Functions
  /// Set FilePath
  void setShortestPathFileName(std::string name);

  /// Set FilePath
  void setLazySearchTreeFileName(std::string name);

  /// set FilePath
  void setEdgeEvaluationsFileName(std::string name);

  /// set FilePath
  void setFrontierNodeDataFileName(std::string name);

  ///////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////

  // Internal evaluation methods
  /// Number of edges evaluated thus far.
  std::size_t getNumEdgeEvaluations() const;

  /// Number of edges rewired thus far.
  std::size_t getNumEdgeRewires() const;

  /// Time for edge evaluations.
  double getEdgeEvaluationsTime() const;

  /// Time for search.
  double getSearchTime() const;

  ///////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////

  // OMPL required methods
  /// Set the problem definition and define the start, goal.
  void setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef);

  /// Solve the planning problem.
  ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc);

  /// Setup the planner.
  void setup() override;

  /// Clear the planner setup.
  void clear() override;

private:
  // Planner parameters
  /// The pointer to the OMPL state space.
  const ompl::base::StateSpacePtr mSpace;

  /// The fixed roadmap over which the search is done.
  Graph graph;

  /// Roadmap
  boost::shared_ptr<utils::RoadmapFromFile<Graph, VPStateMap, utils::StateWrapper, EPLengthMap, EPPriorMap>> roadmapPtr;

  /// Lookahead.
  double mLookahead;

  /// Path to the roadmap.
  std::string mRoadmapFileName;

  /// Connection Radius [Strategy].
  double mConnectionRadius;

  /// Resolution to evaluate edges.
  double mCheckRadius;

  ///////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////

  // Internal evaluation variables
  /// Number of edges evaluated.
  std::size_t mNumEdgeEvals{0u};

  /// Time for edge evaluations
  double mEdgeEvaluationsTime;

  /// Number of edges rewired.
  std::size_t mNumEdgeRewires{0u};

  /// Time for search
  double mSearchTime;

  /// Time for logging
  double mLogTime{0};

  /// Cost of optimal path.
  double mBestPathCost{std::numeric_limits<double>::infinity()};

  /// Track iterations
  std::size_t mIteration{0u};

  /// Filename for logging shortest paths
  std::string mShortestPathsFileName{""};

  /// Filename for logging lazy search tree
  std::string mLazySearchTreeFileName{""};

  /// Filename for logging edge evaluations
  std::string mEdgeEvaluationsFileName{""};

  /// Filename for logging edge evaluations
  std::string mFrontierNodeDataFileName{""};

  ///////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////

  // Internal evaluation functions
  /// Log the shortest path
  void logPath(std::vector<Vertex> path);

  /// Log the lazy search tree
  void logLazySearchTree();

  /// Log edge evaluation
  void logEdgeEvaluation(Vertex u, Vertex v, int result);

  /// Log qFrontier nodes data
  template<class TF>
  void logFrontierNodeData(TF &qFrontier);

  ///////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////

  // Planner helpers
  /// Source vertex.
  Vertex mStartVertex;

  /// Goal vertex.
  Vertex mGoalVertex;

  /// Set of vertices to be rewired.
  unorderedSet mSetRewire;

  /// Edge evaluation resolution manager.
  utils::BisectPerm mBisectPermObj;

  ///////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////

  // Supplementary Functions
  /// Given a new edge, initialize the embedded configurations
  /// along the edge using the resolution of the underlying space.
  /// This is called during problem setup.
  /// \param[in] e The edge ID to initialize with configurations
  void initializeEdgePoints(const Edge& e);

  /// Evaluate an edge to determine its collision status
  /// and assign it to the underlying property of the edge.
  /// \param[in] The edge ID to check for
  /// \return True if edge is valid
  bool evaluateEdge(const Edge& e);

  /// Return the edge between the two vertices
  /// \param[in] source Source vertex
  /// \param[in] target Target vertex
  Edge getEdge(Vertex u, Vertex v) const;

  /// Returns g-value of vertex.
  /// \param[in] vertex Vertex
  double estimateCostToCome(Vertex vertex) const;

  /// Returns heursitic value of vertex.
  /// \param[in] vertex Vertex
  double heuristicFunction(Vertex vertex) const;

  /// Returns f-value of vertex.
  /// \param vertex Vertex
  double estimateTotalCost(Vertex vertex) const;

  /// Returns the path from leaf vertex to border vertex.
  /// \param[in] vertex Vertex on the frontier
  std::vector<Vertex> pathToBorder(Vertex vertex) const;

  /// Construct the solution.
  /// \param[in] start Start vertex
  /// \param[in] goal Goal vertex
  ompl::base::PathPtr constructSolution(const Vertex& start, const Vertex& goal) const;

  ///////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////

  // Main Algorithm Functions
  /// Extend the lazy band.
  /// param[t]  queue with custom comparator
  /// param[in] qExtend priority queue of vertices to extend
  /// param[in] qFrontier priority queue of leaf vertices
  template<class TF>
  void extendLazyBand(TF &qExtend, TF &qFrontier);

  /// Update the lazy band.
  /// param[t]  queue with custom comparator
  /// param[in] qUpdate priority queue of vertices to update
  /// param[in] qExtend priority queue of vertices to extend
  /// param[in] qFrontier priority queue of leaf vertices
  template<class TG, class TF>
  void updateLazyBand(TG &qUpdate, TF &qExtend, TF &qFrontier);

  /// Rewire the lazy band if a collision has occured.
  /// param[t]  queue with custom comparator
  /// param[in] qExtend priority queue of vertices to rewire
  /// param[in] qExtend priority queue of vertices to extend
  /// param[in] qFrontier priority queue of leaf vertices
  template<class TG, class TF>
  void rewireLazyBand(TG &qRewire, TF &qExtend, TF &qFrontier);

  /// Evaluate edges along given path.
  /// param[t] TG Queue with custom comparator
  /// param[t] TF Queue with custom comparator
  /// param[in] path Sequence of vertices along the path.
  /// param[in] qUpdate Priority queue of vertices to update
  /// param[in] qRewire Priority queue of vertices to rewire
  /// param[in] qExtend Priority queue of vertices to extend
  /// param[in] qFrontier Priority queue of leaf vertices
  template<class TG, class TF>
  bool evaluatePath(std::vector<Vertex> path, TG &qUpdate, TG &qRewire, TF &qExtend, TF &qFrontier);

}; // class LRAstar

} // namespace LRAstar
#endif // LRASTAR_LRASTAR_HPP_
