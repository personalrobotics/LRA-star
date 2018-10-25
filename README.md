## Lazy Receding-Horizon A*

Implementation of [LRA*][LRAstar], a lazy search framework that balances edge evaluations with search effort to reduce total planning time. The framework generalizes existing lazy search algorithms [LazySP][lazySP] and [LWA*][LWAstar] using a tunable parameter, lookahead.

Search Algorithm implemented in OMPL, also using Boost Graph Library.

### Dependencies:
1. C++11 or higher
2. cmake
3. OMPL
4. Boost Graph Library
5. OpenCV (optional to display final solution)

The CMakeLists.txt file supports catkin tools. Once you have created and initialized your workspace, 
you should be able to build the package by running `catkin build LRAstar`.

LRAstar has been implemented as an OMPL planner and can be invoked on a `RealVectorStateSpace`.
The planner returns the shortest path on the roadmap graph it is planning on.

------

### Examples:
The executables [`example2D`, `example2D_image`] can be found under `devel/lib/LRA-star/`.
The executables expect five command-line arguments that might be of interest:
1. Path to graph (-g option) 
	- `example2D` defaults to data/graphs/halton2D.graphml
	- `example2D_image` defaults to data/graphs/halton2D.graphml
2. Path to obstacle (-o option) 
	- `example2D` defaults to data/obstacles/Forest2D.txt (requires text format)
	- `example2D_image` defaults to data/obstacles/OneWall2D.png (requires image format)
3. Lookahead (-l option) 
	- Set to 1 for LWA*
	- Set to -1 for LazySP
	- Set to any integer value for LRA* with lookahead
4. Source (2D) between 0 and 1 (-s option) 
5. Target (2D) between 0 and 1 (-t option) 

From the root folder of the catkin workspace:
1. `./devel/lib/LRA-star/example2D -s 0.1 0.1 -t 0.9 0.9 -l -1`
2. `./devel/lib/LRA-star/example2D_image -s 0.1 0.1 -t 0.9 0.9 -l -1`

[LRAstar]: https://personalrobotics.cs.washington.edu/publications/mandalika2018lrastarfull.pdf
[lazySP]: https://personalrobotics.ri.cmu.edu/files/courses/16843/notes/lazysp/lazysp-2016.pdf
[LWAstar]: http://www.roboticsproceedings.org/rss10/p33.pdf
