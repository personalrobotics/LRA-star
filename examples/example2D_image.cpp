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

// OpenCV libraries
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Custom header files
#include "LRAstar/LRAstar.hpp"

namespace po = boost::program_options;

/// Check if the point is within defined hyperrectangle
/// This is bound to the stateValidityChecker of the ompl StateSpace
/// \param[in] image Obstacle image (grayscale)
/// \param[in] state The ompl state to check for validity
/// \return True if the state is collision-free
bool isPointValid(cv::Mat image, const ompl::base::State *state)
{
  // Obtain the state values
  double* values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;

  // Get the required point on the map
  int numberOfRows = image.rows;
  int numberOfColumns = image.cols;
  double x_point = values[0]*numberOfColumns;
  double y_point = (1 - values[1])*numberOfRows;
  cv::Point point((int)x_point, (int)y_point);

  // Collision Check
  int intensity = (int)image.at<uchar>(point.y, point.x);
  if (intensity == 0) // Pixel is black
    return false;

  return true;
}

/// Displays path
/// \param[in] obstacleFile The file with obstacles stored
/// \param[in] path OMPL path
void displayPath(std::string obstacleFile,
                 boost::shared_ptr<ompl::geometric::PathGeometric> path)
{
  // Get state count
  int pathSize = path->getStateCount();

  // Obtain the image matrix
  cv::Mat image = cv::imread(obstacleFile, 1);
  int numberOfRows = image.rows;
  int numberOfColumns = image.cols;

  for (int i = 0; i < pathSize - 1; ++i)
  {
    auto uState = path->getState(i);
    auto vState = path->getState(i+1);
    double* u = uState->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    double* v = vState->as<ompl::base::RealVectorStateSpace::StateType>()->values;

    cv::Point uPoint((int)(u[0]*numberOfColumns), (int)((1 - u[1])*numberOfRows));
    cv::Point vPoint((int)(v[0]*numberOfColumns), (int)((1 - v[1])*numberOfRows));

    cv::line(image, uPoint, vPoint, cv::Scalar(255, 0, 0), 3);
  }

  cv::imshow("Solution Path", image);
  cv::waitKey(0);
}

/// Makes an OMPL state out of given x and y coordinates
/// \param[in] space State space
/// \param[in] x X-Coordinate
/// \param[in] y Y-Coordinate
/// \return OMPL state corresponding to (x, y)
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
      ("type,o", po::value<std::string>()->required(), "one_wall/two_wall")
      ("index,i", po::value<std::string>()->required(), "world index")
      ("source,s", po::value<std::vector<float> >()->multitoken(), "source configuration")
      ("target,t", po::value<std::vector<float> >()->multitoken(), "target configuration")
      ("lookahead,l", po::value<double>()->default_value(1.0), "Lazy Lookahead")
      ("display,d", po::bool_switch()->default_value(false), "Display or Save")
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
  std::string obstacle_type(vm["type"].as<std::string>());
  std::string obstacle_index(vm["index"].as<std::string>());
  std::vector<float> source(vm["source"].as<std::vector< float> >());
  std::vector<float> target(vm["target"].as<std::vector< float> >());
  bool display(vm["display"].as<bool>());

  // Build obstacle location
  std::string default_obstacle_location = "/home/adityavk/research-ws/src/planning_dataset/sanjiban_data/";
  std::string obstacle_file = default_obstacle_location + obstacle_type + "/environment_images/world_" + obstacle_index + ".png";

  // Build the graph location
  std::string default_graph_location = "/home/adityavk/research-ws/src/planning_dataset/sanjiban_data/";
  std::string graph_file = default_graph_location + obstacle_type + "/graph_priors.graphml";

  // Define the state space: R^2
  boost::shared_ptr<ompl::base::RealVectorStateSpace> space(
                                                      new ompl::base::RealVectorStateSpace(2));
  space->as<ompl::base::RealVectorStateSpace>()->setBounds(0.0, 1.0);
  space->setLongestValidSegmentFraction(0.1 / space->getMaximumExtent());
  space->setup();

  // Space Information
  cv::Mat image = cv::imread(obstacle_file, 0);
  std::function<bool(const ompl::base::State*)> isStateValid = std::bind(isPointValid, image, std::placeholders::_1);
  ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
  si->setStateValidityChecker(isStateValid);
  si->setup();

  // Problem Definition
  ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
  pdef->addStartState(make_state(space, source[0], source[1]));
  pdef->setGoalState(make_state(space, target[0], target[1]));

  // Setup planner
  LRAstar::LRAstar planner(si, graph_file, lookahead);

  planner.setup();
  planner.setProblemDefinition(pdef);

  // Solve the motion planning problem
  ompl::base::PlannerStatus status;
  status = planner.solve(ompl::base::plannerNonTerminatingCondition());

  if (display)
  {
    // Display path and specify path size
    if (status == ompl::base::PlannerStatus::EXACT_SOLUTION)
    {
      auto path = boost::dynamic_pointer_cast<ompl::geometric::PathGeometric>(
                                                              pdef->getSolutionPath());
      displayPath(obstacle_file, path);
    }
  }
  else
  {
    // Get planner data if required
    std::cout << "Exiting Cleanly" << std::endl;
  }

  return 0;
}
