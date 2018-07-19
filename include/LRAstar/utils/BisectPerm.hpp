#ifndef LRASTAR_UTILS_BISECTPERM_HPP_
#define LRASTAR_UTILS_BISECTPERM_HPP_

#include <vector>
#include <map>

namespace LRAstar {
namespace utils {

/// Generates a Van Der Corput sequence ordering for states to check along an edge.
class BisectPerm
{
public:
  /// Constructor
  BisectPerm();

  /// Destructor
  ~BisectPerm(void);

  /// Returns a map from integer index to fractional position along edge.
  /// For an edge that has n states, we generate a sequence of n fractional
  /// positions along the edge to check for collision, based on Van Der Corput Sequences
  /// We start at 1/2, then 1/4, and 3/4 and so on upto n states.
  /// \param[in] n The number of states along the edge
  const std::vector<std::pair<int,int>>& get(int n);

private:
  // TODO (avk): docstring for this parameter.
  std::map<int, const std::vector< std::pair<int,int>>> mCache;
};

} // namespace utils
} // namespace LRAstar

#endif // LRASTAR_UTILS_BISECTPERM_HPP_
