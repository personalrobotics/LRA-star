/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, University of Washington
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the University of Washington nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Authors: Aditya Vamsikrishna Mandalika */

#include "LRAstar.hpp"

#include <algorithm>        // std::reverse
#include <cmath>            // pow, sqrt
#include <set>              // std::set
#include <assert.h>         // Debug
#include <chrono>           // record time

namespace LRAstar
{

LRAstar::LRAstar(const ompl::base::SpaceInformationPtr &si)
  : ompl::base::Planner(si, "LRAstar")
  , mSpace(si->getStateSpace())
  , mRoadmapFileName("")
  , mLookahead(1.0)
  , mGreediness(1.0)
  , mConnectionRadius(1.0)
  , mCheckRadius(0.4*mSpace->getLongestValidSegmentLength())
{
  // Register my setting callbacks.
  Planner::declareParam<double>("lookahead", this, &LRAstar::setLookahead, &LRAstar::getLookahead, "1.0:1.0:Infinity");
  Planner::declareParam<double>("greediness", this, &LRAstar::setGreediness, &LRAstar::getGreediness, "1.0:1.0:Lookahead");
  Planner::declareParam<std::string>("roadmapFilename", this, &LRAstar::setRoadmapFileName, &LRAstar::getRoadmapFileName);
}

LRAstar::LRAstar(const ompl::base::SpaceInformationPtr &si,
  const std::string& roadmapFileName,
  double lookahead,
  double greediness)
  : ompl::base::Planner(si, "LRAstar")
  , mSpace(si->getStateSpace())
  , mRoadmapFileName(roadmapFileName)
  , mLookahead(lookahead)
  , mGreediness(greediness)
  , mConnectionRadius(0.4*mSpace->getLongestValidSegmentLength())
  , mCheckRadius(0.5*mSpace->getLongestValidSegmentLength())
{
  if (mRoadmapFileName == "")
    throw std::invalid_argument("Provide a non-empty path to roadmap.");

  if (mLookahead <= 1.0)
    throw std::invalid_argument("Lookahead should be greater than unity.");

  if (mGreediness > mLookahead)
    throw std::invalid_argument("Greediness is bounded above by lookahead.");
}

LRAstar::~LRAstar()
{
  // Do nothing.
}

// ===========================================================================================
void LRAstar::setup()
{
  ompl::base::Planner::setup();

  roadmapPtr = boost::shared_ptr<utils::RoadmapFromFile<Graph, VPStateMap, utils::StateWrapper, EPLengthMap>>
                (new utils::RoadmapFromFile<Graph, VPStateMap, utils::StateWrapper, EPLengthMap>
                (mSpace, mRoadmapFileName));

  roadmapPtr->generate(graph, get(&VProp::state, graph),
                          get(&EProp::length, graph));

  // Set default vertex values.
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi)
  {
    graph[*vi].costToCome = std::numeric_limits<double>::infinity();
    graph[*vi].lazyCostToCome = std::numeric_limits<double>::infinity();
    graph[*vi].heuristic = std::numeric_limits<double>::infinity();
    graph[*vi].inLazyBand = false;
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
    graph[*vi].budgetToExtend = 0;
    graph[*vi].inLazyBand = false;
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
  graph[mGoalVertex].costToCome = 0;
  graph[mGoalVertex].lazyCostToCome = 0;
  graph[mGoalVertex].heuristic = heuristicFunction(mStartVertex);
  graph[mStartVertex].parent = -1;
  graph[mStartVertex].inLazyBand = false;
  graph[mStartVertex].visited = false;
  graph[mStartVertex].status = CollisionStatus::FREE;

  mGoalVertex = boost::add_vertex(graph);
  graph[mGoalVertex].state = goalState;
  graph[mGoalVertex].costToCome = std::numeric_limits<double>::infinity();
  graph[mGoalVertex].lazyCostToCome = std::numeric_limits<double>::infinity();
  graph[mGoalVertex].heuristic = 0;
  graph[mGoalVertex].inLazyBand = false;
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

  graph[mStartVertex].visited = true;
  qExtend.insert(mStartVertex);

  extendLazyBand(qExtend, qFrontier);

  std::vector<Vertex> path;
  while((!qFrontier.empty() && !solutionFound) || !qExtend.empty())
  {
    Vertex vTop = *qFrontier.begin();
    qFrontier.erase(qFrontier.begin());
    assert(graph[vTop].budgetToExtend() == mLookahead || vTop == mGoalVertex);

    path = pathToBorder(vTop);
    bool goalFound = evaluatePath(path, qUpdate, qRewire, qExtend, qFrontier);

    if(goalFound)
    {
        OMPL_INFORM("Solution Found!");
        solutionFound = true;
        break;
    }
    updateLazyBand(qUpdate, qExtend, qFrontier);
    rewireLazyBand(qRewire, qExtend, qFrontier);
    extendLazyBand(qExtend, qFrontier);
  }

  if(solutionFound)
  {
    mBestPathCost = estimateCostToCome(mGoalVertex);
    pdef_->addSolutionPath(constructSolution(mStartVertex, mGoalVertex));

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
        Edge uv;
        bool edgeExists;
        boost::tie(uv, edgeExists) = edge(u, v, graph);
        assert(edgeExists);
        double edgeLength = graph[uv].length;

        if(graph[uv].status == CollisionStatus::FREE)
        {
          if(graph[v].visited == false)
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


            // If the budget of "v" is alpha, mark entire subsequent subtree as not visited
            // These nodes don't need rewiring since shortest path to them must be via v
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
          double budget = graph[v].budgetToExtend;
          if (budget == mLookahead)
          {
            assert(qFrontier.find(v) == qFrontier.end());
            qFrontier.emplace(v);
          }
          else // budget < mLookahead
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
  /*assert(mSetRewire.empty());

  // 1. Collect all the nodes that need to be rewired
  while(!qRewire.empty())
  {
    Vertex v = *qRewire.begin();
    qRewire.erase(qRewire.begin());

    // Add all the children of the current node to qRewire and empty the children vector
    std::vector<Vertex>& children = g[v].node.children();
    for(auto iterV = children.begin(); iterV != children.end(); ++iterV)
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
    g[v].node.updateParent(v);
    g[v].node.updateCost(std::numeric_limits<double>::max());
    g[v].node.updateLazyCost(0);
    g[v].node.updateBudget(0);

    // Mark it as not visited
    g[v].visited = false;
  }

  // Record number of edge rewires
  mNumEdgeRewires += mSetRewire.size();

  // 2. Assign the nodes keys
  for(auto iterS = mSetRewire.begin(); iterS != mSetRewire.end(); ++iterS)
  {
    Vertex v = *iterS;
    if(g[v].status == CollisionStatus::BLOCKED)
      continue;

    NeighborIter ni, ni_end;
    for(boost::tie(ni, ni_end) = adjacent_vertices(v, g); ni != ni_end; ++ni)
    {
      Vertex u = *ni; // Possible parent

      if(g[u].status == CollisionStatus::BLOCKED)
        continue;

      if(u == mGoalVertex) // Do not rewire to goal vertex
        continue;

      if(g[u].node.cost() == std::numeric_limits<double>::max())
        continue;

      if (g[u].visited == false)
        continue;

      if(g[u].node.budget() == mLookahead) // parent should have budget
        continue;

      if(qExtend.find(u) != qExtend.end())
        continue;

      assert (mSetRewire.find(u) == mSetRewire.end());
      assert(v != g[u].node.parent()); // entire subtree should have been collected

      Edge uv;
      bool edgeExists;
      boost::tie(uv, edgeExists) = edge(u, v, g);
      assert(edgeExists);
      double edgeLength = g[uv].length;

      if(g[uv].status == CollisionStatus::FREE)
      {
        if(estimateCostToCome(v) > estimateCostToCome(u) + edgeLength ||
          (estimateCostToCome(v) == estimateCostToCome(u) + edgeLength && u < g[v].node.parent()))
        {
          g[v].node.updateCost(g[u].node.cost());
          g[v].node.updateLazyCost(g[u].node.lazyCost() + edgeLength);
          g[v].node.updateParent(u);
          g[v].node.updateBudget(g[u].node.budget() + 1);
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

    if(u == g[u].node.parent())
      continue;

    if(estimateTotalCost(g[u].node.parent()) >= cReference)
    {
      qExtend.emplace(g[u].node.parent());
      continue;
    }

    // Since valid parent is found, mark as visited
    g[u].visited = true;

    // Let the parent know of its new child
    Vertex p = g[u].node.parent();
    std::vector<Vertex>& children = g[p].node.children();
    children.emplace_back(u);

    if(g[u].node.budget() < mLookahead && u != mGoalVertex)
    {
      assert(qExtend.find(u) == qExtend.end());
      assert(qExtend.find(p) == qExtend.end());
      assert(g[u].node.children().empty());

      qExtend.emplace(u);
    }

    if(g[u].node.budget() == mLookahead || u == mGoalVertex)
    {
      assert(qFrontier.find(u) == qFrontier.end());
      qFrontier.emplace(u);
      continue;
    }

    NeighborIter ni, ni_end;
    for (boost::tie(ni, ni_end) = adjacent_vertices(u, g); ni != ni_end; ++ni)
    {
      Vertex v = *ni;

      if(g[v].status == CollisionStatus::BLOCKED)
        continue;

      // Vertex needs to be in set to update
      if(qRewire.find(v) == qRewire.end())
        continue;

      assert(v != p); // Enforce prevention of loops - parent should not be in Qrewire

      Edge uv;
      bool edgeExists;
      boost::tie(uv, edgeExists) = edge(u, v, g);
      assert(edgeExists);
      double edgeLength = g[uv].length;

      if(g[uv].status == CollisionStatus::FREE)
      {
        if(estimateCostToCome(v) > estimateCostToCome(u) + edgeLength ||
          (estimateCostToCome(v) == estimateCostToCome(u) + edgeLength && u < g[v].node.parent()))
        {
          if(qExtend.find(u) != qExtend.end())
          {
            qRewire.erase(v);
            g[v].visited = false;
            g[v].node.updateCost(std::numeric_limits<double>::max());
            g[v].node.updateParent(v);
            qRewire.emplace(v);
            continue;
          }

          qRewire.erase(v);

          g[v].node.updateCost(g[u].node.cost());
          g[v].node.updateLazyCost(g[u].node.lazyCost() + edgeLength);
          g[v].node.updateParent(u);
          g[v].node.updateBudget(g[u].node.budget() + 1);

          assert(g[u].node.budget() <  mLookahead);
          assert(g[v].node.budget() <= mLookahead);

          qRewire.emplace(v);
        }
      }
    }
  }
  mSetRewire.clear();*/
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
    greediness = mLookahead;
  else
    greediness = mGreediness;

  bool isLeaf;
  for(auto iterV = path.rbegin(); iterV != path.rbegin() + greediness && iterV != path.rend(); ++iterV)
  {
    isLeaf = false;

    Vertex u = *iterV;
    Vertex v = *(iterV + 1);

    // Determine the edge length
    Edge uv;
    bool edgeExists;
    boost::tie(uv, edgeExists) = edge(u, v, graph);
    assert(edgeExists);
    double edgeLength = graph[uv].length;

    // Actual Collision Check
    if (!evaluateEdge(uv))
    {
      std::vector<Vertex>& children = graph[u].children;
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

      qRewire.emplace(v);
      break;
    }

    else
    {
      if (graph[v].budgetToExtend == mLookahead)
      {
        auto iterQ = qFrontier.find(v);
        if(iterQ != qFrontier.end())
          qFrontier.erase(iterQ);
        isLeaf = true;
      }

      graph[v].budgetToExtend = 0;
      graph[v].lazyCostToCome = 0;
      graph[v].costToCome = graph[u].costToCome + edgeLength;

      assert(qFrontier.find(v) == qFrontier.end());
      assert(qExtend.find(v) == qExtend.end());

      if (isLeaf)
      {
        assert(qExtend.find(u) == qExtend.end());
        assert(qFrontier.find(u) == qFrontier.end());

        assert(qExtend.find(v) == qExtend.end());
        qExtend.emplace(v);
      }

      qUpdate.emplace(v);
      if(v == mGoalVertex)
        return true;
    }
  }
  return false;
}

// ===========================================================================================
// Setters
void LRAstar::setLookahead(double lookahead)
{
  mLookahead = lookahead;
}

void LRAstar::setGreediness(double greediness)
{
  mGreediness = greediness;
}

void LRAstar::setRoadmapFileName(const std::string& roadmapFileName)
{
  mRoadmapFileName = roadmapFileName;
}

void LRAstar::setConnectionRadius(double connectionRadius)
{
  mConnectionRadius = connectionRadius;
}

// ===========================================================================================
// Getters
double LRAstar::getLookahead() const
{
  return mLookahead;
}

double LRAstar::getGreediness() const
{
  return mGreediness;
}

std::string LRAstar::getRoadmapFileName() const
{
  return mRoadmapFileName;
}

double LRAstar::getConnectionRadius() const
{
  return mConnectionRadius;
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
  // March along edge states with highest resolution
  mNumEdgeEvals++;

  auto validityChecker = si_->getStateValidityChecker();
  
  Vertex startVertex = source(e,graph);
  Vertex endVertex   = target(e,graph);
  auto startState = graph[startVertex].state->state;
  auto endState = graph[endVertex].state->state;

  auto nStates = graph[e].edgeStates.size();

  bool checkResult;
  
  // Evaluate Start and End States [we only assume states in self-collision are pruned out]
  checkResult = validityChecker->isValid(startState);
  if(!checkResult)
  {
    graph[startVertex].status = CollisionStatus::BLOCKED;
    return checkResult;
  }
  checkResult = validityChecker->isValid(endState);
  if(!checkResult)
  {
    graph[endVertex].status = CollisionStatus::BLOCKED;
    return checkResult;
  }

  // Evaluate the States in between
  for(unsigned int i = 1; i < nStates-1; i++)
  {
    checkResult = validityChecker->isValid(graph[e].edgeStates[i]->state);
    if(!checkResult)
    {
      graph[e].status = CollisionStatus::BLOCKED;
      graph[e].length = std::numeric_limits<double>::max();
      break;
    }
  }
  return checkResult;
}

// ===========================================================================================
using Vertex = LRAstar::Vertex;
std::vector<Vertex> LRAstar::pathToBorder(Vertex v) const
{
  std::vector<Vertex> path;
  path.emplace_back(v);
  double currentBudget = graph[v].budgetToExtend;

  while(currentBudget > 0)
  {
    v = graph[v].parent;
    assert(graph[v].budgetToExtend == currentBudget - 1);
    path.emplace_back(v);
    currentBudget = graph[v].budgetToExtend;

    assert(graph[v].visited);
    assert(graph[v].parent != v);
    assert(graph[v].budgetToExtend <= mLookahead);
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

} // namespace LRAstar
