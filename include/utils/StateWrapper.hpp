#ifndef STATE_WRAPPER_HPP_
#define STATE_WRAPPER_HPP_

#include <ompl/base/StateSpace.h>

namespace LRAstar {
namespace utils {

struct StateWrapper
{
  StateWrapper() {}
  StateWrapper(ompl::base::StateSpacePtr space):
      space(space), state(space->allocState()) {}
  ~StateWrapper() { space->freeState(this->state); }

  const ompl::base::StateSpacePtr space;
  ompl::base::State *state;
};
typedef boost::shared_ptr<StateWrapper> StateWrapperPtr;

} // namespace utils
} // namespace LRAstar

#endif // STATE_WRAPPER_H_
