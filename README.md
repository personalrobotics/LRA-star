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

To run an example script, build the package. The executable should be within the devel/lib/LRAstar folder
of your catkin workspace.

The planner expects the following:
1. Path to the roadmap file the planner has to plan on [required]
2. Path to the obstacle file [required]
3. The source configuration [required]
4. The target configuration [required]
5. The lookahead [optional; default = 1]
6. The greediness [optional; default = 1]

The example script has been set up. It generates an executable `example2D` in the devel/lib/LRAstar folder.

To run the planner: [from the catkin workspace]

`./devel/lib/LRA-star/example2D -f src/LRA-star/scripts/halton2D.graphml -o src/LRA-star/scripts/2DForest.txt -s 0.1 0.1 -t 0.9 0.9 -l 1`

Note:
-f -> Sets the path to the roadmap file (in the scripts folder)
-o -> Sets the path to the obstacle file (in the scripts folder)
-s -> Sets the source
-t -> Sets the target
-l -> Sets the lookahead

As one increases the lookahead, the number of edge evaluations (printed to screen) decreases while
number of edge rewires "typically" increases.

The obstacle file can be visualized using the python file in the scripts folder. Note that the halton graph provided
has 2000 vertices with a connection radius of 0.04. The same connection radius has been used to augment the source and 
target configurations into the graph.