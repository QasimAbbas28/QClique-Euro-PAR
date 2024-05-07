#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <string>
#include <algorithm>
#include <thread>
#include <cmath>
#include "vertex-edge.h"

using namespace std;

class Clique {
  vector<int> vertices;
  vector<Edge> edges;
  float totalWeight;

  public:
    Clique() {
      totalWeight = 0;
    }

    void addVertex(int label) {
      vertices.push_back(label);
    }

    void updateWeight(float newWeight) {
      totalWeight = totalWeight+newWeight;
    }

    float getTotalWeight() {
      return totalWeight;
    }

    vector<int> getVertices() {
      return vertices;
    }
};