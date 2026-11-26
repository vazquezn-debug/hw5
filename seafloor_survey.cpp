/*
 * STUDENT CODE - OPTIONAL - EDIT, REPLACE, OR DELETE
 */

#include "seafloor_survey.h"
#include <cstdlib>
#include <algorithm>

void SeafloorSurvey::recordDebris(Location location)
{
  /* TOY APPROACH*/
  if(location.robot >= static_cast<int>(debrisbyRobot.size())){
    debrisbyRobot.resize(location.robot +1);
  }
  debrisbyRobot[location.robot].push_back(location.segment);
}

int SeafloorSurvey::getSignalStrength(Location location, int RobotCount) const
{
  /* TOY APPROACH */
  int signal = 0;

  int leftRobo = (location.robot-1+RobotCount) % RobotCount;
  int rightRobo = (location.robot + 1) % RobotCount;
        
  int RobotsToChk[3] = {location.robot,leftRobo, rightRobo};

  int min_seg = location.segment - SIGNAL_RANGE;
  int max_seg = location.segment + SIGNAL_RANGE;

  for(int i = 0; i < 3; i++){
    int robot = RobotsToChk[i];
    if(robot >= static_cast<int>(debrisbyRobot.size())){
      continue;
    }
    const std::vector<int> &segments = debrisbyRobot[robot];

    auto first = std::lower_bound(segments.begin(), segments.end(), min_seg);
    auto last = std::upper_bound(segments.begin(), segments.end(), max_seg);
    signal += static_cast<int>(last - first);
  }
  return signal;

  
}

/* EXAMPLE
bool SeafloorSurvey::hasDebris() const
{
  return !debrisLocations.empty();
}
*/