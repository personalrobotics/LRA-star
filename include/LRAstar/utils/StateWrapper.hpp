#ifndef LRASTAR_UTILS_STATE_WRAPPER_HPP_
#define LRASTAR_UTILS_STATE_WRAPPER_HPP_

#include <ompl/base/StateSpace.h>

namespace LRAstar {
namespace utils {

struct StateWrapper
{
  StateWrapper()
  {
    // Do nothing.
  }

  StateWrapper(ompl::base::StateSpacePtr space)
    : space(space)
    , state(space->allocState())
  {
    // Do nothing.
  }

  ~StateWrapper()
  {
    space->freeState(this->state);
  }

  /// The OMPL statespace operating on.
  const ompl::base::StateSpacePtr space;

  /// The OMPL state that is being wrapped.
  ompl::base::State *state;
};

/// Alias declaration.
typedef boost::shared_ptr<StateWrapper> StateWrapperPtr;

} // namespace utils
} // namespace LRAstar

#endif // LRASTAR_UTILS_STATE_WRAPPER_HPP_
