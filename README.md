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
The executables [`example2D`, `example2D_image`] can be found under `devel/lib/LRA-star/`.
The executables expect three command-line arguments that might be of interest:
1. Path to graph 
	`example2D` defaults to data/graphs/halton.graphml
	`example2D_image` defaults to data/graphs/halton.graphml
2. Path to obstacle
	`example2D` defaults to data/obstacles/Forest2D.txt (requires text format)
	`example2D_image` defaults to data/obstacles/one_wall.png (requires image format)
3. Lookahead
	Set to 1 for LWA*
	Set to -1 for LazySP