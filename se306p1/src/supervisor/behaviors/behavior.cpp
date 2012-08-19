#include "behavior.h"
#include "../supervisor.h"

#include <memory>
#include <vector>

namespace se306p1 {
Behavior::Behavior(Supervisor &sup) : supervisor_(sup) {
}

Behavior::~Behavior() {
}

void Behavior::MoveNodesToDests(
    const std::vector<std::shared_ptr<Robot> > &nodesIn,
    const std::vector<Pose> &posesIn) {
  std::vector<std::shared_ptr<Robot> > nodes = nodesIn;
  std::vector<Pose> poses = posesIn;

  while (poses.size()) {
    double longestDist = -1;
    int longestNodeIndex = 0;
    int longestPoseIndex = 0;

    for (size_t nodeIndex = 0; nodeIndex < nodes.size(); nodeIndex++) {
      double dist = std::numeric_limits<double>::max();
      int destIndex = 0;
      std::shared_ptr<Robot> node = nodes[nodeIndex];
      for (size_t posesIndex = 0; posesIndex < poses.size(); posesIndex++) {
        double testDist =
            abs((node->pose_.position_ - poses[posesIndex].position_)
                .LengthSquared());
        if (testDist < dist) {
          dist = testDist;
          destIndex = posesIndex;
        }
      }
      if (dist > longestDist) {
        longestDist = dist;
        longestNodeIndex = nodeIndex;
        longestPoseIndex = destIndex;
      }
    }
    nodes[longestNodeIndex]->Go(poses[longestPoseIndex], false);
    nodes.erase(nodes.begin() + longestNodeIndex);
    poses.erase(poses.begin() + longestPoseIndex);
  }
}
}
