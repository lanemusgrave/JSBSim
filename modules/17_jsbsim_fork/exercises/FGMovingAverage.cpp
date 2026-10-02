// Module 17 exercise A: skeleton.  Copy to external/jsbsim/src/models/flight_control/
// Model it on FGDeadBand.cpp.  Then register the component:
//   1. src/models/flight_control/CMakeLists.txt   add the .cpp to SOURCES and the .h to HEADERS
//   2. src/models/FGFCS.cpp                         #include it; add an `else if` for "moving_average"
//   3. src/models/flight_control/FGFCSComponent.cpp give it a Type name (otherwise "UNKNOWN")
// Build:  cmake --build external/jsbsim/build --target JSBSim -j4
// Check:  python modules/17_jsbsim_fork/solutions/check_moving_average.py

#include "FGMovingAverage.h"
#include "models/FGFCS.h"                 // without it: "invalid use of incomplete type FGFCS"
#include "input_output/FGXMLElement.h"
#include "input_output/FGLog.h"

using namespace std;

namespace JSBSim {

FGMovingAverage::FGMovingAverage(FGFCS* fcs, Element* element)
  : FGFCSComponent(fcs, element)          // parses name, <input>, <output>, <clipto>
{
  CheckInputNodes(1, 1, element);

  // TODO: read <samples> (element->FindElementValueAsNumber), reject < 1
  //       (throw an XMLLogException(element)), size the buffer

  bind(element, fcs->GetPropertyManager().get());   // creates the output property
}

FGMovingAverage::~FGMovingAverage() {}

bool FGMovingAverage::Run(void)
{
  Input = InputNodes[0]->getDoubleValue();

  // TODO: first call after a reset: fill the buffer with Input (no start-up transient)
  // TODO: store Input, advance the write index, Output = mean of the buffer

  Clip();        // applies <clipto>
  SetOutput();   // writes Output to the component's property (and any <output>)
  return true;
}
}
