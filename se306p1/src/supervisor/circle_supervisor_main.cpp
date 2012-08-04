#include "circle_supervisor.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "supervisor");
  se306p1::CircleSupervisor s;
  s.Run();
}