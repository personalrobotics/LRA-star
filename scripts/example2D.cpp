// Standard C++ libraries
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <queue>

// Boost libraries
#include <boost/shared_ptr.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/function.hpp>
#include <boost/program_options.hpp>

// OMPL base libraries
#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>

// Custom header files
#include "LRAstar/LRAstar.hpp"

namespace po = boost::program_options;

/// Check if the point is within defined hyperrectangle
/// This is bound to the stateValidityChecker of the ompl StateSpace
/// \param[in] obstacles The file with obstacles stored
/// \param[in] state The ompl state to check for validity
/// \return True if the state is collision-free
bool isPointValid(std::string obstacleFile, const ompl::base::State *state)
{
  // Read the Obstacle File
  std::ifstream infile(obstacleFile);
  double lowerX, lowerY, upperX, upperY;
  while (infile >> lowerX >> lowerY >> upperX >> upperY)
  {
    double* values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    if(values[0] >= lowerX && values[0] <= upperX && values[1] >= lowerY && values[1] <= upperY)
      return false;
  }
  return true;
}

ompl::base::ScopedState<ompl::base::RealVectorStateSpace>
make_state(const ompl::base::StateSpacePtr space, double x, double y)
{
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace>state(space);
  double* values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  values[0] = x;
  values[1] = y;
  return state;
}

int main(int argc, char *argv[])
{
  po::options_description desc("2D Map Planner Options");
  desc.add_options()
      ("help,h", "produce help message")
      ("roadmapfile,f", po::value<std::string>()->required(), "Path to Graph")
      ("obstaclefile,o", po::value<std::string>()->required(), "Path to Obstacles File")
      ("source,s", po::value<std::vector<float> >()->multitoken(), "source configuration")
      ("target,t", po::value<std::vector<float> >()->multitoken(), "target configuration")
      ("lookahead,l", po::value<double>()->default_value(1.0), "Lazy Lookahead")
      ("greediness,g", po::value<double>()->default_value(1.0), "Greediness")
  ;

  // Read arguments
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
      std::cout << desc << std::endl;
      return 1;
  }

  double lookahead(vm["lookahead"].as<double>());
  double greediness(vm["greediness"].as<double>());
  std::string graph_file(vm["roadmapfile"].as<std::string>());
  std::string obstacle_file(vm["obstaclefile"].as<std::string>());
  std::vector<float> source(vm["source"].as<std::vector< float> >());
  std::vector<float> target(vm["target"].as<std::vector< float> >());

  // Define the state space: R^2
  boost::shared_ptr<ompl::base::RealVectorStateSpace> space(new ompl::base::RealVectorStateSpace(2));
  space->as<ompl::base::RealVectorStateSpace>()->setBounds(0.0, 1.0);
  space->setLongestValidSegmentFraction(0.1 / space->getMaximumExtent());
  space->setup();

  // Space Information
  std::function<bool(const ompl::base::State*)> isStateValid = std::bind(isPointValid, obstacle_file, std::placeholders::_1);
  ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
  si->setStateValidityChecker(isStateValid);
  si->setup();

  // Problem Definition
  ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
  pdef->addStartState(make_state(space, source[0], source[1]));
  pdef->setGoalState(make_state(space, target[0], target[1]));

  // Setup planner
  LRAstar::LRAstar planner(si, graph_file, lookahead, greediness);
  planner.setup();
  planner.setProblemDefinition(pdef);

  // Solve the motion planning problem
  ompl::base::PlannerStatus status;
  status = planner.solve(ompl::base::plannerNonTerminatingCondition());

  // Obtain required data if plan was successful
  if(status == ompl::base::PlannerStatus::EXACT_SOLUTION)
  {
    // Get planner data if required
    return 0;
  }

  return 0;
}
