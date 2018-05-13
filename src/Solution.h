#ifndef SRC_SOLUTION_H_
#define SRC_SOLUTION_H_

#include <vector>

class Solution {

  public:
    double steer;
    double throttle;
    std::vector<double> xvals;
    std::vector<double> yvals;

    Solution()
    {
      steer = 0.0;
      throttle = 0.0;
    }

    ~Solution(){}

};
#endif /* SRC_SOLUTION_H_ */
