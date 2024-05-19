/*
 * This file is part of QClique.
 *
 * QClique is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * QClique is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with QClique. If not, see <https://www.gnu.org/licenses/>.
 */

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