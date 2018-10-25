/* Authors: Aditya Vamsikrishna Mandalika */

#include "LRAstar/LRAstar.hpp"

#include <algorithm>        // std::reverse
#include <cmath>            // pow, sqrt
#include <set>              // std::set
#include <assert.h>         // debug
#include <fstream>          // log
#include <chrono>           // time

namespace LRAstar
{

LRAstar::LRAstar(const ompl::base::SpaceInformationPtr &si)
  : ompl::base::Planner(si, "LRAstar")
  , mSpace(si->getStateSpace())
  , mRoadmapFileName("")
  , mLookahead(1.0)
  , mConnectionRadius(1.0)
  , mCheckRadius(0.005*mConnectionRadius)
{
  // Register my setting callbacks.
  Planner::declareParam<std::string>("roadmapFilename", this, &LRAstar::setRoadmapFileName, &LRAstar::getRoadmapFileName);
  Planner::declareParam<double>("lookahead", this, &LRAstar::setLookahead, &LRAstar::getLookahead, "0.0:1.0");
}

LRAstar::LRAstar(const ompl::base::SpaceInformationPtr &si,
  const std::string& roadmapFileName,
  double lookahead)
  : ompl::base::Planner(si, "LRAstar")
  , mSpace(si->getStateSpace())
  , mRoadmapFileName(roadmapFileName)
  , mLookahead(lookahead)
  , mConnectionRadius(0.04)
  , mCheckRadius(0.005*mConnectionRadius)
{

  // In case lookahead is set to -1
  setLookahead(mLookahead);

  if (mRoadmapFileName == "")
    throw std::invalid_argument("Provide a non-empty path to roadmap.");

  if (mLookahead < 1.0)
    throw std::invalid_argument("Lookahead should be greater than or equal to 1");
}

LRAstar::~LRAstar()
{
  // Do nothing.
}

// ===========================================================================================
void LRAstar::setup()
{
  // Check if already setup.
  if (static_cast<bool>(ompl::base::Planner::setup_))
    return;

  ompl::base::Planner::setup();

  roadmapPtr = boost::shared_ptr<utils::RoadmapFromFile<Graph, VPStateMap, utils::StateWrapper, EPLengthMap, EPPriorMap>>
                (new utils::RoadmapFromFile<Graph, VPStateMap, utils::StateWrapper, EPLengthMap, EPPriorMap>
                (mSpace, mRoadmapFileName));

  roadmapPtr->generate(graph,
                       get(&VProp::state, graph),
                       get(&EProp::length, graph),
                       get(&EProp::prior, graph));

  // Set default vertex values.
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi)
  {
    graph[*vi].costToCome = std::numeric_limits<double>::infinity();
    graph[*vi].lazyCostToCome = std::numeric_limits<double>::infinity();
    graph[*vi].heuristic = std::numeric_limits<double>::infinity();
    graph[*vi].budgetToExtend = 0;
    graph[*vi].visited = false;
    graph[*vi].status = CollisionStatus::FREE;
  }

  // Set default edge values.
  EdgeIter ei, ei_end;
  for (boost::tie(ei, ei_end) = edges(graph); ei != ei_end; ++ei)
  {
    graph[*ei].isEvaluated = false;
    graph[*ei].status = CollisionStatus::FREE;
    initializeEdgePoints(*ei);
  }

  mBestPathCost = std::numeric_limits<double>::infinity();
}

// ===========================================================================================
void LRAstar::clear()
{
  // Set default vertex values.
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi)
  {
    graph[*vi].costToCome = std::numeric_limits<double>::infinity();
    graph[*vi].lazyCostToCome = std::numeric_limits<double>::infinity();
    graph[*vi].heuristic = std::numeric_limits<double>::infinity();
    graph[*vi].budgetToExtend = 0.0;
    graph[*vi].visited = false;
    graph[*vi].status = CollisionStatus::FREE;
  }

  // Set default edge values.
  EdgeIter ei, ei_end;
  for (boost::tie(ei, ei_end) = edges(graph); ei != ei_end; ++ei)
  {
    graph[*ei].isEvaluated = false;
    graph[*ei].status = CollisionStatus::FREE;
    initializeEdgePoints(*ei);
  }

  // Internal evaluation variables.
  mBestPathCost = std::numeric_limits<double>::infinity();
  mNumEdgeEvals = 0;
  mNumEdgeRewires = 0;

  // Helper containers.
  mSetRewire.clear();
}

// ===========================================================================================
void LRAstar::setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef)
{
  // Make sure we setup the planner first.
  if (!static_cast<bool>(ompl::base::Planner::setup_))
  {
    setup();
  }

  ompl::base::Planner::setProblemDefinition(pdef);

  utils::StateWrapperPtr startState(new utils::StateWrapper(mSpace));
  mSpace->copyState(startState->state, pdef_->getStartState(0));

  utils::StateWrapperPtr goalState(new utils::StateWrapper(mSpace));
  mSpace->copyState(goalState->state, pdef_->getGoal()->as<ompl::base::GoalState>()->getState());

  auto validityChecker = si_->getStateValidityChecker();

  if(!validityChecker->isValid(startState->state))
    throw ompl::Exception("Start configuration is in collision!");
  if(!validityChecker->isValid(goalState->state))
    throw ompl::Exception("Goal configuration is in collision!");

  // Add start and goal vertices to the graph
  mStartVertex = boost::add_vertex(graph);
  graph[mStartVertex].state = startState;

  mGoalVertex = boost::add_vertex(graph);
  graph[mGoalVertex].state = goalState;

  // Assign default values
  graph[mStartVertex].costToCome = 0;
  graph[mStartVertex].lazyCostToCome = 0;
  graph[mStartVertex].heuristic = heuristicFunction(mStartVertex);
  graph[mStartVertex].budgetToExtend = 0;
  graph[mStartVertex].parent = -1;
  graph[mStartVertex].visited = false;
  graph[mStartVertex].status = CollisionStatus::FREE;

  graph[mGoalVertex].costToCome = std::numeric_limits<double>::infinity();
  graph[mGoalVertex].lazyCostToCome = std::numeric_limits<double>::infinity();
  graph[mGoalVertex].heuristic = 0;
  graph[mGoalVertex].budgetToExtend = 0;
  graph[mGoalVertex].visited = false;
  graph[mGoalVertex].status = CollisionStatus::FREE;

  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi)
  {
    double startDist = mSpace->distance(graph[*vi].state->state, startState->state);
    double goalDist  = mSpace->distance(graph[*vi].state->state, goalState->state);

    if (startDist < mConnectionRadius)
    {
      if(mStartVertex == *vi)
        continue;
      std::pair<LRAstar::Edge,bool> newEdge = boost::add_edge(mStartVertex, *vi, graph);
      graph[newEdge.first].length = startDist;
      graph[newEdge.first].prior = 1.0;
      graph[newEdge.first].isEvaluated = false;
      graph[newEdge.first].status = CollisionStatus::FREE;
      initializeEdgePoints(newEdge.first);
    }

    if (goalDist < mConnectionRadius)
    {
      if(mGoalVertex == *vi)
        continue;
      std::pair<LRAstar::Edge,bool> newEdge = boost::add_edge(mGoalVertex, *vi, graph);
      graph[newEdge.first].length = goalDist;
      graph[newEdge.first].prior = 1.0;
      graph[newEdge.first].isEvaluated = false;
      graph[newEdge.first].status = CollisionStatus::FREE;
      initializeEdgePoints(newEdge.first);
    }
  }
}

// ===========================================================================================
ompl::base::PlannerStatus LRAstar::solve(const ompl::base::PlannerTerminationCondition & ptc)
{
  // Priority Function: g-value
  auto cmpGValue = [&](const Vertex& left, const Vertex& right)
  {
    double estimateLeft = estimateCostToCome(left);
    double estimateRight = estimateCostToCome(right);

    if (estimateRight - estimateLeft > 0)
      return true;
    if (estimateLeft - estimateRight > 0)
      return false;
    if (left < right)
      return true;
    else
      return false;
  };

  // Priority Function: f-value
  auto cmpFValue = [&](const Vertex& left, const Vertex& right)
  {
    double estimateLeft = estimateTotalCost(left);
    double estimateRight = estimateTotalCost(right);

    if (estimateRight - estimateLeft > 0)
      return true;
    if (estimateLeft - estimateRight > 0)
      return false;
    if (left < right)
      return true;
    else
      return false;
  };

  std::set<Vertex, decltype(cmpGValue)> qUpdate(cmpGValue);
  std::set<Vertex, decltype(cmpGValue)> qRewire(cmpGValue);
  std::set<Vertex, decltype(cmpFValue)> qExtend(cmpFValue);
  std::set<Vertex, decltype(cmpFValue)> qFrontier(cmpFValue);

  bool solutionFound = false;

  // Log Time
  std::chrono::time_point<std::chrono::system_clock> startTime{std::chrono::system_clock::now()};

  graph[mStartVertex].visited = true;
  qExtend.insert(mStartVertex);

  extendLazyBand(qExtend, qFrontier);

  std::vector<Vertex> path;
  while((!qFrontier.empty() && !solutionFound) || !qExtend.empty())
  {
    // Log the lazy search tree
    // logLazySearchTree();
    if (mFrontierNodeDataFileName != "")
      logFrontierNodeData(qFrontier);

    Vertex vTop = *qFrontier.begin();
    qFrontier.erase(qFrontier.begin());
    assert(graph[vTop].budgetToExtend == mLookahead || vTop == mGoalVertex);

    path = pathToBorder(vTop);

    // Log the shortest path
    // logPath(path);

    bool goalFound = evaluatePath(path, qUpdate, qRewire, qExtend, qFrontier);

    if (goalFound)
    {
      OMPL_INFORM("Solution Found!");
      solutionFound = true;
      break;
    }
    updateLazyBand(qUpdate, qExtend, qFrontier);
    rewireLazyBand(qRewire, qExtend, qFrontier);
    extendLazyBand(qExtend, qFrontier);

    mIteration++;
  }

  std::chrono::time_point<std::chrono::system_clock> endTime{std::chrono::system_clock::now()};
  std::chrono::duration<double> elapsedSeconds{endTime-startTime};
  mSearchTime = elapsedSeconds.count() - mEdgeEvaluationsTime - mLogTime;

  if(solutionFound)
  {
    mBestPathCost = estimateCostToCome(mGoalVertex);
    pdef_->addSolutionPath(constructSolution(mStartVertex, mGoalVertex));

    OMPL_INFORM("Lookahead:                   %f", (mLookahead == 
                                                    std::numeric_limits<double>::max()) ? -1 : mLookahead);
    OMPL_INFORM("Number of Edges Rewired:     %d", mNumEdgeRewires);
    OMPL_INFORM("Number of Edges Evaluated:   %d", mNumEdgeEvals);
    OMPL_INFORM("Cost of goal:                %f", mBestPathCost);

    return ompl::base::PlannerStatus::EXACT_SOLUTION;
  }

  else
  {
    OMPL_INFORM("Solution NOT Found");
  }
}

// ===========================================================================================
//Extend
template<class TF>
void LRAstar::extendLazyBand(TF &qExtend, TF &qFrontier)
{
  while(!qExtend.empty())
  {
    // Obtain the Top Key in qFrontier for Lazy Extension
    double cReference;
    if(qFrontier.empty())
      cReference = std::numeric_limits<double>::max();
    else
    {
      Vertex vReference = *qFrontier.begin();
      cReference = estimateTotalCost(vReference);
    }
    Vertex u = *qExtend.begin();

    // Lazy Extension
    if(estimateTotalCost(u) >= cReference)
      break;

    qExtend.erase(qExtend.begin());
    assert(graph[u].budgetToExtend < mLookahead);
    assert(graph[u].visited);

    if(graph[u].status == CollisionStatus::BLOCKED)
      continue;

    if(u == mGoalVertex)
      qFrontier.emplace(u);

    else
    {
      NeighborIter ni, ni_end;
      for (boost::tie(ni, ni_end) = adjacent_vertices(u, graph); ni != ni_end; ++ni)
      {
        Vertex v = *ni;

        if(graph[v].status == CollisionStatus::BLOCKED)
          continue;

        // Enforce prevention of loops
        if(v == graph[u].parent)
          continue;

        // Determine the edge length
        Edge uv = getEdge(u, v);
        double edgeLength = graph[uv].length;

        if (graph[uv].status == CollisionStatus::FREE)
        {
          if (graph[v].visited == false)
          {
            graph[v].visited = true;
            assert(qExtend.find(v) == qExtend.end());
            assert(qFrontier.find(v) == qFrontier.end());
          }
          else
          {
            double estimateOld = estimateCostToCome(v);
            double estimateNew = estimateCostToCome(u) + edgeLength;
            Vertex previousParent = graph[v].parent;

            if(estimateOld < estimateNew)
              continue;

            // Tie-Breaking Rule
            if(estimateOld == estimateNew)
            {
              if(previousParent < u)
                continue;
            }

            // else, update the old parent and the subsequent subtree

            // Remove vertex from its current siblings
            std::vector<Vertex>& children = graph[previousParent].children;
            std::size_t indx;
            for(indx = 0; indx != children.size(); ++indx)
            {
              if(children[indx] == v)
                break;
            }
            assert(indx != children.size()); // child has been found

            // Remove child
            if (indx != children.size()-1)
              children[indx] = children.back();
            children.pop_back();

            // Remove old node from any queue
            auto iterQ = qFrontier.find(v);
            if(iterQ != qFrontier.end())
              qFrontier.erase(iterQ);

            iterQ = qExtend.find(v);
            if(iterQ != qExtend.end())
              qExtend.erase(iterQ);


            // If this guy has kids, remove them i.e mark them as not visited.
            // and clear them from queues. They do not need rewiring at this point.
            std::vector<Vertex> subtree = {v};
            while(!subtree.empty())
            {
              auto iterT = subtree.rbegin();
              std::vector<Vertex>& children = graph[*iterT].children;
              subtree.pop_back();

              for(auto iterV = children.begin(); iterV != children.end(); ++iterV)
              {
                graph[*iterV].visited = false;
                subtree.emplace_back(*iterV);

                auto iterQ = qFrontier.find(*iterV);
                if(iterQ != qFrontier.end())
                  qFrontier.erase(iterQ);

                iterQ = qExtend.find(*iterV);
                if(iterQ != qExtend.end())
                  qExtend.erase(iterQ);
              }
              children.clear();
            }
          }

          // Update Vertex Node
          graph[v].parent = u;
          graph[v].costToCome = graph[u].costToCome;
          graph[v].lazyCostToCome = graph[u].lazyCostToCome + edgeLength;
          graph[v].budgetToExtend = graph[u].budgetToExtend + 1;
          graph[v].heuristic = heuristicFunction(v);

          // Add it to its new siblings
          std::vector<Vertex>& children = graph[u].children;
          children.emplace_back(v);

          // Add it to appropriate queue
          if (graph[v].budgetToExtend == mLookahead)
          {
            assert(qFrontier.find(v) == qFrontier.end());
            qFrontier.emplace(v);
          }
          else // budget >= mLookahead
          {
            assert(qExtend.find(v) == qExtend.end());
            qExtend.emplace(v);
          }
        }
      }
    }
  }
}

// ===========================================================================================
// Update
template<class TG, class TF>
void LRAstar::updateLazyBand(TG &qUpdate, TF &qExtend, TF &qFrontier)
{
  while(!qUpdate.empty())
  {
    Vertex u = *qUpdate.begin();
    qUpdate.erase(qUpdate.begin());

    assert(graph[u].budgetToExtend < mLookahead);
    bool isLeaf;

    std::vector<Vertex>& children = graph[u].children;

    for(auto iterV = children.begin(); iterV != children.end(); ++iterV)
    {
      Vertex v = *iterV;
      isLeaf = false;

      if(graph[v].budgetToExtend != 0)
      {
        Edge uv;
        bool edgeExists;
        boost::tie(uv, edgeExists) = edge(u, v, graph);
        assert(edgeExists);
        double edgeLength = graph[uv].length;

        // Remove vertex from qFrontier
        if(graph[v].budgetToExtend == mLookahead)
        {
          auto iterQ = qFrontier.find(v);
          if(iterQ != qFrontier.end())
            qFrontier.erase(iterQ);
          isLeaf = true;
        }

        // Update nodes in qExtend
        auto iterQ = qExtend.find(v);
        if(iterQ != qExtend.end())
        {
          qExtend.erase(iterQ);
          isLeaf = true;
        }

        graph[v].costToCome = graph[u].costToCome;
        graph[v].lazyCostToCome = graph[u].lazyCostToCome + edgeLength;
        graph[v].budgetToExtend = graph[u].budgetToExtend + 1;

        if(isLeaf || v == mGoalVertex)
        {
          assert(qExtend.find(u) == qExtend.end());
          assert(qFrontier.find(u) == qFrontier.end());
          assert(qExtend.find(v) == qExtend.end());
          qExtend.emplace(v);
        }
        assert(graph[v].budgetToExtend < mLookahead);

        qUpdate.emplace(v);
      }
    }
  }
}

// ===========================================================================================
// Rewire
template<class TG, class TF>
void LRAstar::rewireLazyBand(TG &qRewire, TF &qExtend, TF &qFrontier)
{
  // We should have had this cleared
  assert(mSetRewire.empty());

  // 1. Collect all the nodes that need to be rewired
  while (!qRewire.empty())
  {
    Vertex v = *qRewire.begin();
    qRewire.erase(qRewire.begin());

    // Add all the children of the current node to qRewire and empty the children vector
    std::vector<Vertex>& children = graph[v].children;
    for (auto iterV = children.begin(); iterV != children.end(); ++iterV)
    {
      qRewire.emplace(*iterV);
    }
    children.clear();

    // Add the vertex to set
    mSetRewire.insert(v);

    // Remove from qFrontier
    auto iterQ = qFrontier.find(v);
    if(iterQ != qFrontier.end())
      qFrontier.erase(iterQ);
    assert(qFrontier.find(v) == qFrontier.end());

    // Remove from qExtend
    iterQ = qExtend.find(v);
    if(iterQ != qExtend.end())
      qExtend.erase(iterQ);
    assert(qExtend.find(v) == qExtend.end());

    // Assign default values
    graph[v].parent = v;
    graph[v].costToCome = std::numeric_limits<double>::max();
    graph[v].lazyCostToCome = 0;
    graph[v].budgetToExtend = 0;

    // Mark it as not visited
    graph[v].visited = false;
  }

  // Record number of edge rewires
  mNumEdgeRewires += mSetRewire.size();

  // 2. Assign the nodes keys
  for (auto iterS = mSetRewire.begin(); iterS != mSetRewire.end(); ++iterS)
  {
    Vertex v = *iterS;
    if(graph[v].status == CollisionStatus::BLOCKED)
      continue;

    NeighborIter ni, ni_end;
    for (boost::tie(ni, ni_end) = adjacent_vertices(v, graph); ni != ni_end; ++ni)
    {
      Vertex u = *ni; // Possible parent

      if (graph[u].status == CollisionStatus::BLOCKED)
        continue;

      if (u == mGoalVertex) // Do not rewire to goal vertex
        continue;

      if (graph[u].costToCome == std::numeric_limits<double>::max())
        continue;

      if (graph[u].visited == false)
        continue;

      if (graph[u].budgetToExtend == mLookahead) // parent should have budget
        continue;

      if (qExtend.find(u) != qExtend.end())
        continue;

      assert(mSetRewire.find(u) == mSetRewire.end());
      assert(v != graph[u].parent); // entire subtree should have been collected

      Edge uv = getEdge(u, v);
      double edgeLength = graph[uv].length;

      if (graph[uv].status == CollisionStatus::FREE)
      {
        // If we have a better parent than the current, rewire.
        if (estimateCostToCome(v) > estimateCostToCome(u) + edgeLength ||
           (estimateCostToCome(v) == estimateCostToCome(u) + edgeLength && u < graph[v].parent))
        {
          graph[v].costToCome = graph[u].costToCome;
          graph[v].lazyCostToCome = graph[u].lazyCostToCome + edgeLength;
          graph[v].parent = u;
          graph[v].budgetToExtend = graph[u].budgetToExtend + 1;
        }
      }
    }
    qRewire.emplace(v);
  }

  // Obtain the Top Key in qFrontier for Lazy Rewire
  double cReference;
  if(qFrontier.empty())
    cReference = std::numeric_limits<double>::max();
  else
  {
    Vertex vReference = *qFrontier.begin();
    cReference = estimateTotalCost(vReference);
  }

  // 3. Start Rewiring in the cost space
  while(!qRewire.empty())
  {
    Vertex u = *qRewire.begin();
    qRewire.erase(qRewire.begin());

    if (u == graph[u].parent)
      continue;

    if (estimateTotalCost(graph[u].parent) >= cReference)
    {
      qExtend.emplace(graph[u].parent);
      continue;
    }

    // Since valid parent is found, mark as visited
    graph[u].visited = true;

    // Let the parent know of its new child
    Vertex p = graph[u].parent;
    std::vector<Vertex>& children = graph[p].children;
    children.emplace_back(u);

    assert(qExtend.find(u) == qExtend.end());
    assert(qFrontier.find(u) == qFrontier.end());
    assert(graph[u].children.empty());

    if (graph[u].budgetToExtend == mLookahead || u == mGoalVertex)
    {
      qFrontier.emplace(u);
      continue;
    }

    if (graph[u].budgetToExtend < mLookahead)
    {
      qExtend.emplace(u);
    }

    NeighborIter ni, ni_end;
    for (boost::tie(ni, ni_end) = adjacent_vertices(u, graph); ni != ni_end; ++ni)
    {
      Vertex v = *ni;

      if (graph[v].status == CollisionStatus::BLOCKED)
        continue;

      // Vertex needs to be in set to update
      if (qRewire.find(v) == qRewire.end())
        continue;

      assert(v != p); // Enforce prevention of loops - parent should not be in qRewire

      Edge uv = getEdge(u, v);
      double edgeLength = graph[uv].length;

      if(graph[uv].status == CollisionStatus::FREE)
      {
        // If u is a better better than the current one, rewire.
        if(estimateCostToCome(v) > estimateCostToCome(u) + edgeLength ||
          (estimateCostToCome(v) == estimateCostToCome(u) + edgeLength && u < graph[v].parent))
        {
          // If u is not in qExtend, then we will be rewired in extend, so assign default values.
          if (qExtend.find(u) != qExtend.end())
          {
            qRewire.erase(v);
            graph[v].visited = false;
            graph[v].costToCome = std::numeric_limits<double>::max();
            graph[v].parent = v;
            graph[v].budgetToExtend = 0;
            qRewire.emplace(v);
            continue;
          }

          qRewire.erase(v);

          // We found a better parent.
          // TODO (avk): why will the control ever reach here?
          graph[v].costToCome = graph[u].costToCome;
          graph[v].lazyCostToCome = graph[u].lazyCostToCome + edgeLength;
          graph[v].parent = u;
          graph[v].budgetToExtend = graph[u].budgetToExtend + 1;

          assert(graph[u].budgetToExtend < mLookahead);
          assert(graph[v].budgetToExtend <= mLookahead);

          qRewire.emplace(v);
        }
      }
    }
  }
  mSetRewire.clear();
}

// ===========================================================================================
// Evaluate
template<class TG, class TF>
bool LRAstar::evaluatePath(std::vector<Vertex> path, TG &qUpdate, TG &qRewire, TF &qExtend, TF &qFrontier)
{
  assert(path.size() != 1);

  // Increase greediness to lookahead if goal is on current path
  int greediness;
  if(path[0] == mGoalVertex)
    greediness = path.size()-1;
  else
    greediness = 1;

  bool isLeaf;
  for(auto iterV = path.rbegin(); iterV != path.rbegin() + greediness && iterV != path.rend(); ++iterV)
  {
    isLeaf = false;

    Vertex u = *iterV;
    Vertex v = *(iterV + 1);

    // Determine the edge length
    Edge uv = getEdge(u, v);

    // Actual Collision Check
    if (!evaluateEdge(uv)) // Edge is in collision
    {
      // Remove v from u's children. v shall be updated
      // appropriately in the rewire function.
      std::vector<Vertex>& children = graph[u].children;
      std::size_t indx;
      for (indx = 0; indx != children.size(); ++indx)
      {
        if (children[indx] == v)
          break;
      }
      assert(indx != children.size()); // child has been found

      // Remove child
      if (indx != children.size() - 1)
          children[indx] = children.back();
      children.pop_back();

      qRewire.emplace(v);
      break;
    }

    else
    {
      double edgeLength = graph[uv].length;

      // Cleanup queues.
      // If it was in qFrontier, it no longer belongs
      // since it can be extended. It cannot be in qExtend since qExtend is either empty
      // or has vertices along paths that are not going to be selected (branch and bound).
      if (graph[v].budgetToExtend == mLookahead)
      {
        auto iterQ = qFrontier.find(v);
        if (iterQ != qFrontier.end())
          qFrontier.erase(iterQ);
        isLeaf = true;
      }
      // Therefore v should not be in any of extend/frontier.
      assert(qExtend.find(v) == qExtend.end());
      assert(qFrontier.find(v) == qFrontier.end());

      graph[v].budgetToExtend = 0;
      graph[v].lazyCostToCome = 0;
      graph[v].costToCome = graph[u].costToCome + edgeLength;

      if (isLeaf)
      {
        // u should have been extended + should not have been frontier
        assert(qExtend.find(u) == qExtend.end());
        assert(qFrontier.find(u) == qFrontier.end());

        qExtend.emplace(v);
      }
      else // is not a leaf, therefore subtree needs to be updated.
      {
        qUpdate.emplace(v);
      }

      if (v == mGoalVertex)
        return true;
    }
  }
  return false;
}

// ===========================================================================================
// Setters
void LRAstar::setLookahead(double lookahead)
{
  if (lookahead == -1)
    mLookahead = std::numeric_limits<double>::max();
  else
    mLookahead = lookahead;
}

void LRAstar::setRoadmapFileName(const std::string& roadmapFileName)
{
  mRoadmapFileName = roadmapFileName;
}

void LRAstar::setConnectionRadius(double connectionRadius)
{
  mConnectionRadius = connectionRadius;
}

void LRAstar::setCheckRadius(double checkRadius)
{
  mCheckRadius = checkRadius;
}

void LRAstar::setShortestPathFileName(std::string name)
{
  std::string sourceName = "/home/adityavk/research-ws/src/planning_dataset/results/forward/search/";
  mShortestPathsFileName = sourceName + std::to_string((int)mLookahead) + '_' + name;
}

void LRAstar::setLazySearchTreeFileName(std::string name)
{
  std::string sourceName = "/home/adityavk/research-ws/src/planning_dataset/results/forward/search/";
  mLazySearchTreeFileName = sourceName + std::to_string((int)mLookahead) + '_' + name;
}

void LRAstar::setEdgeEvaluationsFileName(std::string name)
{
  std::string sourceName = "/home/adityavk/research-ws/src/planning_dataset/results/forward/search/";
  mEdgeEvaluationsFileName = sourceName + std::to_string((int)mLookahead) + '_' + name;
}

void LRAstar::setFrontierNodeDataFileName(std::string name)
{
  std::string sourceName = "/home/adityavk/research-ws/src/planning_dataset/results/forward/search/";
  mFrontierNodeDataFileName = sourceName + std::to_string((int)mLookahead) + '_' + name;
}

// ===========================================================================================
// Getters
double LRAstar::getLookahead() const
{
  return mLookahead;
}

std::string LRAstar::getRoadmapFileName() const
{
  return mRoadmapFileName;
}

double LRAstar::getConnectionRadius() const
{
  return mConnectionRadius;
}

double LRAstar::getCheckRadius() const
{
  return mCheckRadius;
}

LRAstar::Vertex LRAstar::getStartVertex() const
{
  return mStartVertex;
}

LRAstar::Vertex LRAstar::getGoalVertex() const
{
  return mGoalVertex;
}

double LRAstar::getBestPathCost() const
{
  return mBestPathCost;
}

std::size_t LRAstar::getNumEdgeEvaluations() const
{
  return mNumEdgeEvals;
}

std::size_t LRAstar::getNumEdgeRewires() const
{
  return mNumEdgeRewires;
}

double LRAstar::getEdgeEvaluationsTime() const
{
  return mEdgeEvaluationsTime;
}

double LRAstar::getSearchTime() const
{
  return mSearchTime;
}

// ===========================================================================================
ompl::base::PathPtr LRAstar::constructSolution(const Vertex &start, const Vertex &goal) const
{
  std::set<Vertex> seen;

  ompl::geometric::PathGeometric *path = new ompl::geometric::PathGeometric(si_);
  Vertex v = goal;
  while (v != start)
  {
    if (seen.find(v) != seen.end())
    {
      OMPL_ERROR("infinite loop");
      break;
    }

    seen.insert(v);
    path->append(graph[v].state->state);
    v = graph[v].parent;
  }

  if (v == start)
  {
    path->append(graph[start].state->state);
  }
  path->reverse();
  return ompl::base::PathPtr(path);
}

// ===========================================================================================
void LRAstar::initializeEdgePoints(const Edge& e)
{
  auto startState = graph[source(e,graph)].state->state;
  auto endState = graph[target(e,graph)].state->state;

  unsigned int nStates = static_cast<unsigned int>(std::floor(graph[e].length / (2.0*mCheckRadius)));
  
  // Just start and goal
  if(nStates < 2u)
  {
    nStates = 2u;
  }

  graph[e].edgeStates.resize(nStates);

  for(unsigned int i = 0; i < nStates; i++)
  {
    graph[e].edgeStates[i].reset(new utils::StateWrapper(mSpace));
  }

  const std::vector<std::pair<int,int>>& order = mBisectPermObj.get(nStates);

  for(unsigned int i = 0; i < nStates; i++)
  {
    mSpace->interpolate(startState, endState,
      1.0*(1+order[i].first)/(nStates+1), graph[e].edgeStates[i]->state);
  }
}

// ===========================================================================================
bool LRAstar::evaluateEdge(const LRAstar::Edge& e)
{
  // Log Time
  std::chrono::time_point<std::chrono::system_clock> startEvaluationTime{std::chrono::system_clock::now()};

  // March along edge states with highest resolution
  mNumEdgeEvals++;
  graph[e].isEvaluated = true;

  auto validityChecker = si_->getStateValidityChecker();
  
  Vertex startVertex = source(e,graph);
  Vertex endVertex   = target(e,graph);
  auto startState = graph[startVertex].state->state;
  auto endState = graph[endVertex].state->state;

  auto nStates = graph[e].edgeStates.size();

  bool checkResult = true;
  
  // Evaluate Start and End States [we only assume states in self-collision are pruned out]
  if (checkResult && !validityChecker->isValid(startState))
  {
    graph[startVertex].status = CollisionStatus::BLOCKED;
    graph[e].status = CollisionStatus::BLOCKED;
    graph[e].length = std::numeric_limits<double>::max();
    checkResult = false;
  }

  if (checkResult && !validityChecker->isValid(endState))
  {
    graph[endVertex].status = CollisionStatus::BLOCKED;
    graph[e].status = CollisionStatus::BLOCKED;
    graph[e].length = std::numeric_limits<double>::max();
    checkResult = false;
  }

  if (checkResult)
  {
    // Evaluate the States in between
    for (unsigned int i = 1; i < nStates-1; i++)
    {
      if(!validityChecker->isValid(graph[e].edgeStates[i]->state))
      {
        graph[e].status = CollisionStatus::BLOCKED;
        graph[e].length = std::numeric_limits<double>::max();
        checkResult = false;
        break;
      }
    }
  }

  std::chrono::time_point<std::chrono::system_clock> endEvaluationTime{std::chrono::system_clock::now()};
  std::chrono::duration<double> elapsedSeconds{endEvaluationTime-startEvaluationTime};
  mEdgeEvaluationsTime += elapsedSeconds.count();

  // logEdgeEvaluation(startVertex, endVertex, checkResult);
  return checkResult;
}

// ===========================================================================================
using Edge = LRAstar::Edge;
Edge LRAstar::getEdge(Vertex u, Vertex v) const
{
  Edge uv;
  bool edgeExists;
  boost::tie(uv, edgeExists) = edge(u, v, graph);

  return uv;
}

// ===========================================================================================
using Vertex = LRAstar::Vertex;
std::vector<Vertex> LRAstar::pathToBorder(Vertex v) const
{
  std::vector<Vertex> path;
  path.emplace_back(v);

  Vertex u = graph[v].parent;
  Edge e = getEdge(u, v);

  while (!graph[e].isEvaluated)
  {
    path.emplace_back(u);

    if (u == mStartVertex)
      break;

    v = u;
    u = graph[u].parent;
    e = getEdge(u, v);
  }

  return path;
}

// ===========================================================================================
double LRAstar::estimateCostToCome(Vertex v) const
{
  return graph[v].costToCome + graph[v].lazyCostToCome;
}

double LRAstar::heuristicFunction(Vertex v) const
{
  return mSpace->distance(graph[v].state->state, graph[mGoalVertex].state->state);
}

double LRAstar::estimateTotalCost(Vertex v) const
{
  return estimateCostToCome(v) + heuristicFunction(v);
}

// ===========================================================================================
// Internal evaluation functions
void LRAstar::logPath(std::vector<Vertex> path)
{
  // Log Time
  std::chrono::time_point<std::chrono::system_clock> startTime{std::chrono::system_clock::now()};

  Vertex u = path[0];
  while (u != -1)
  {
    std::ofstream logFile;
    double *Uvals = (graph[u].state->state)->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    logFile.open(mShortestPathsFileName, std::ios_base::app);
    logFile << mIteration << " " << Uvals[0] << " " << Uvals[1] << std::endl;
    logFile.close();

    u = graph[u].parent;
  }

  std::chrono::time_point<std::chrono::system_clock> endTime{std::chrono::system_clock::now()};
  std::chrono::duration<double> elapsedSeconds{endTime-startTime};
  mLogTime += elapsedSeconds.count();
}

void LRAstar::logLazySearchTree()
{
  // Log Time
  std::chrono::time_point<std::chrono::system_clock> startTime{std::chrono::system_clock::now()};

  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi)
  {
    if (graph[*vi].visited && *vi != mStartVertex)
    {
      Vertex u = graph[*vi].parent;
      Vertex v = *vi;

      double *Uvals = (graph[u].state->state)->as<ompl::base::RealVectorStateSpace::StateType>()->values;
      double *Vvals = (graph[v].state->state)->as<ompl::base::RealVectorStateSpace::StateType>()->values;

      std::ofstream logFile;
      logFile.open(mLazySearchTreeFileName, std::ios_base::app);
      logFile << mIteration << " " << Uvals[0] << " " << Uvals[1] << " " <<
                                      Vvals[0] << " " << Vvals[1] << std::endl;
      logFile.close();
    }
  }
  std::chrono::time_point<std::chrono::system_clock> endTime{std::chrono::system_clock::now()};
  std::chrono::duration<double> elapsedSeconds{endTime-startTime};
  mLogTime += elapsedSeconds.count();
}

void LRAstar::logEdgeEvaluation(Vertex u, Vertex v, int result)
{
  // Log Time
  std::chrono::time_point<std::chrono::system_clock> startTime{std::chrono::system_clock::now()};

  std::ofstream logFile;
  double *Uvals = (graph[u].state->state)->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  double *Vvals = (graph[v].state->state)->as<ompl::base::RealVectorStateSpace::StateType>()->values;

  logFile.open(mEdgeEvaluationsFileName, std::ios_base::app);
  logFile << mIteration << " " << Uvals[0] << " " << Uvals[1] << " " <<
                                  Vvals[0] << " " << Vvals[1] << " " <<
                                  result << std::endl;
  logFile.close();

  std::chrono::time_point<std::chrono::system_clock> endTime{std::chrono::system_clock::now()};
  std::chrono::duration<double> elapsedSeconds{endTime-startTime};
  mLogTime += elapsedSeconds.count();
}

template<class TF>
void LRAstar::logFrontierNodeData(TF &qFrontier)
{
  for (auto iterQ = qFrontier.begin(); iterQ != qFrontier.end(); ++iterQ)
  {
    std::ofstream logFile;
    logFile.open(mFrontierNodeDataFileName, std::ios_base::app);

    Vertex v = *iterQ;

    int depth = 0;
    while (v != mStartVertex)
    {
      Vertex u = graph[v].parent;
      Edge uv = getEdge(u, v);
      if (!graph[uv].isEvaluated)
      {
        depth++;
        v = u;
      }
      else
        break;
    }

    logFile << mIteration << " " << depth << std::endl;
    logFile.close();
  }
}

} // namespace LRAstar
