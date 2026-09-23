#include "FilterDerivative.h"
#include "main.h"

float FilterDerivative::input( float inVal, long us ) {
  long thisUS = HAL_GetTick() * 1000.0;
  
  if ((us < 0) || (us < LastUS))
    thisUS = HAL_GetTick() * 1000.0; // get current time
  else
    thisUS = us;

  float dt = 1e-6*float(thisUS - LastUS);   // cast to float here, for math
  LastUS = thisUS;                          // update this now
  
  Derivative = (inVal-LastInput) / dt;
    
  LastInput = inVal;
  return output();
}
  
float FilterDerivative::output() { return Derivative; }

