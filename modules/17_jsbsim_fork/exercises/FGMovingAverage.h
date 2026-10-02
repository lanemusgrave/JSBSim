// Module 17 exercise A: skeleton.  Copy to external/jsbsim/src/models/flight_control/
// and fill in the TODOs.  Model it on FGDeadBand.h (same directory).
// Solution: ../solutions/0001-Add-moving_average-FCS-component.patch

#ifndef FGMOVINGAVERAGE_H
#define FGMOVINGAVERAGE_H

#include <vector>

#include "FGFCSComponent.h"

namespace JSBSim {

class FGFCS;
class Element;

/** Moving-average (boxcar) filter: output = mean of the last N inputs.

    <moving_average name="fcs/q-avg">
      <input> velocities/q-rad_sec </input>
      <samples> 12 </samples>
    </moving_average>
*/
class FGMovingAverage : public FGFCSComponent
{
public:
  FGMovingAverage(FGFCS* fcs, Element* element);
  ~FGMovingAverage();

  bool Run(void) override;
  // TODO: which FGFCSComponent virtual must you override so that run_ic()
  //       and reset_to_initial_conditions() clear the filter's memory?

private:
  std::vector<double> buffer;
  // TODO: state for the circular buffer (write index, "has it been filled yet?")
};
}
#endif
