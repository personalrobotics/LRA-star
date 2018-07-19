#include "LRAstar/utils/BisectPerm.hpp"

namespace LRAstar {
namespace utils {

BisectPerm::BisectPerm()
{
  // Do nothing.
}

BisectPerm::~BisectPerm()
{
  // Do nothing.
}

const std::vector<std::pair<int, int>>& BisectPerm::get(int n)
{
  std::map<int, const std::vector<std::pair<int,int>>>::iterator it;
  it = mCache.find(n);
  if (it != mCache.end())
    return it->second;

  int i;
  int last_true;
  int max_i;
  int max_val;
  std::vector<std::pair<int,int>> perm;
  std::vector<bool> done(n, false);
  std::vector<int> dist(n);

  for (;;)
  {
    last_true = -1;
    for (i=0; i<n; i++)
    {
      if (done[i]) last_true = i;
      dist[i] = (i-last_true);
    }
    last_true = n;
    for (i=n-1; i>=0; i--)
    {
      if (done[i]) last_true = i;
      dist[i] = (last_true-i) < dist[i] ? (last_true-i) : dist[i];
    }
    max_val = 0;
    max_i = 0;
    for (i=0; i<n; i++) if (max_val < dist[i])
    {
      max_val = dist[i];
      max_i = i;
    }
    if (!max_val)
      break;
    perm.push_back(std::make_pair(max_i,max_val));
    done[max_i] = true;
  }

  mCache.insert(std::make_pair(n,perm));
  return mCache[n];
}

} // namespace utils
} // namespace LRAstar
