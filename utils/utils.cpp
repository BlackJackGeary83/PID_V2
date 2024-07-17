#include "utils.hpp"

double heading_error(double bearing, double current_heading) {
   double error = current_heading - bearing;
   if (error > 180)
      error -= 360;
   if (error < -180)
      error += 360;
   return error;
}
