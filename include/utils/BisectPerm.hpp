#ifndef UTIL_BISECTPERM_HPP_
#define UTIL_BISECTPERM_HPP_

#include <vector>
#include <map>

namespace LRAstar {
namespace utils {

//! Generates a Van Der Corput sequence ordering for states to check along an edge.

class BisectPerm
{
public:
  /// Constructor
  BisectPerm();

  /// Destructor
  ~BisectPerm(void);

  /// For an edge that has n states, we generate a sequence of n fractional
  /// positions along the edge to check for collision, based on Van Der Corput Sequences
  /// We start at 1/2, then 1/4, and 3/4 and so on upto n states
  /// \param[in] n The number of states along the edge
  /// \return A map from integer index to fractional position along edge.
  const std::vector<std::pair<int,int>>& get(int n);

private:
  std::map<int, const std::vector< std::pair<int,int>>> mCache;
};

} // namespace utils
} // namespace LRAstar

#endif // UTIL_BISECTPERM_HPP_
