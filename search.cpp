/*
 * STUDENT CODE - EDIT THIS FILE
 */

#include "search.h"

#include <algorithm>
#include <queue>
#include <utility>

Search::Search(Mothership &mothership)
    : mothership(mothership)
// , robotSelector(mothership, seafloorSurvey) /* EXAMPLE
{
}

SearchResult Search::findShipwreck()
{
  /* TOY APPROACH - Depth First Search - (CHANGE OR REPLACE) */
  const std::vector<Location> &abysses = mothership.getAbysses();

  int robotCnt = mothership.getRobotCount();
  std::priority_queue<std::pair<int, Location>> Loc_to_investigate;
  

  std::vector<int>lstSafeSegment(robotCnt, TETHER_LENGTH);


  for(const Location&abyss : abysses){
      lstSafeSegment[abyss.robot] = std::min(lstSafeSegment[abyss.robot], abyss.segment - 1);
  }

  for (int robot = 0; robot < robotCnt; ++robot)
  { 
    int fst_target = std::min(MAX_SEGMENTS_PER_DRIVE, lstSafeSegment[robot]);

    if(fst_target > mothership.getRobotPosition(robot)){
      Loc_to_investigate.push({0, {robot, fst_target}});
    }

  }

  
  while(!Loc_to_investigate.empty()){
    auto best_Cand = Loc_to_investigate.top();
    Loc_to_investigate.pop();

    Location loc = best_Cand.second;
    int lastSafeSegment = lstSafeSegment[loc.robot];
    if(loc.segment > lastSafeSegment){
      continue;
    }
    int currPos = mothership.getRobotPosition(loc.robot);
    if(loc.segment <= currPos){
      continue;
    }
    if(mothership.getActiveRobot() != loc.robot){
      mothership.switchToRobot(loc.robot);
    }
    while(mothership.getRobotPosition(loc.robot) < loc.segment){
      int distance = loc.segment - mothership.getRobotPosition(loc.robot);
      int driveDist = std::min(MAX_SEGMENTS_PER_DRIVE, distance);
      DriveResult result = mothership.driveActiveRobot(driveDist);
      if(result.reason == DriveStopReason::ShipwreckFound){
        return SearchResult{true, Location{loc.robot, mothership.getRobotPosition(loc.robot)}};
      }
      if(result.reason == DriveStopReason::DebrisFound){
        int debris_Segment = mothership.getRobotPosition(loc.robot);
        seafloorSurvey.recordDebris({loc.robot, debris_Segment});

        int leftRobo = (loc.robot-1+robotCnt) % robotCnt;
        int rightRobo = (loc.robot + 1) % robotCnt;
        
        Location left_cand{leftRobo, debris_Segment};
        Location right_cand{rightRobo, debris_Segment};

        int left_signal = seafloorSurvey.getSignalStrength(left_cand, robotCnt);
        int right_signal = seafloorSurvey.getSignalStrength(right_cand, robotCnt);

        Loc_to_investigate.push({left_signal, left_cand});
        Loc_to_investigate.push({right_signal, right_cand});
      }
    }
    int currPosition = mothership.getRobotPosition(loc.robot);

    int nextTarget = std::min(currPosition + MAX_SEGMENTS_PER_DRIVE, lastSafeSegment);

    if(nextTarget > currPosition){
      Location nextLocation{loc.robot, nextTarget};
      int nextsignal = seafloorSurvey.getSignalStrength(nextLocation, robotCnt);
      Loc_to_investigate.push({nextsignal, nextLocation});
    }
      
  }

  return SearchResult{false, Location{}};
}
