#include "utils.hpp"

double heading_error(double bearing, double current_heading) {
   double error = bearing - current_heading;
   if (error > 180)
      error -= 360;
   if (error < -180)
      error += 360;
   return error;
}
