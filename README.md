Lazy Receding-Horizon A*

Search Algorithm implemented in OMPL, also using Boost Graph Library.

Dependencies:
1. C++11 or higher
2. cmake
3. OMPL
4. Boost Graph Library

The CMakeLists.txt file supports catkin tools. Once you have created and initialized your workspace, 
you should be able to build the package by running `catkin build LRAstar`.

LRAstar has been implemented as an OMPL planner and can be invoked on a `RealVectorStateSpace`.
The planner returns the shortest path on the roadmap graph it is planning on.

------

Example:
