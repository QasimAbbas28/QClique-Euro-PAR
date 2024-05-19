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

using namespace std;

// class Graph;
class Edge;
class Vertex;
class Clique;

class Clique {
  vector<string> vertices;
  float totalWeight;

  public:
    Clique() {
      totalWeight = 0;
    }

    void addVertex(string label) {
      vertices.push_back(label);
    }

    void updateWeight(float newWeight) {
      totalWeight = totalWeight+newWeight;
    }

    float getTotalWeight() {
      return totalWeight;
    }

    vector<string> getVertices() {
      return vertices;
    }
};

class Edge {
  Vertex * vertex1;
  Vertex * vertex2;
public:
  Vertex* getV1() const {return vertex1;}
  Vertex* getV2() const {return vertex2;}
  
  void setV1(Vertex * v){vertex1=v;}
  void setV2(Vertex * v){vertex2=v;}
  Edge(Vertex* v1, Vertex* v2){vertex1=v1;vertex2=v2;}
};

class Vertex {
  string label;
  float weight;
  vector<Edge *> edgesLeavingMe; //neighbours of this vertex
  bool visited;
public:
  string getLabel() const {
    return label;
  }
  float getWeight() const {
    return weight;
  }
  vector<Edge*> getEdges()const{
    return edgesLeavingMe;
  }

  void setWeight(float w){weight=w;}

  Edge * getEdgeTo(string d){
    for (vector<Edge *>::iterator it = edgesLeavingMe.begin(); it != edgesLeavingMe.end(); ++it){
      if ((*it)->getV2()->getLabel()==d){
    return (*it);
      }
    }
    return 0;
  }
  void setVisited(bool v){visited=v;}
  bool getVisited() {return visited;}
  void addEdge(Edge * e){edgesLeavingMe.push_back(e);}
  void removeEdge(Edge * e){
    edgesLeavingMe.erase(remove(edgesLeavingMe.begin(),edgesLeavingMe.end(),e),edgesLeavingMe.end());
  }

  void removeEdgeTo(string l){
    Edge * e = getEdgeTo(l);
    removeEdge(e);
  }

  int getDegree() {
    return edgesLeavingMe.size();
  }

  Vertex(string l, float w){
    label=l; 
    visited=false;
    weight=w;
  }
};

class UnDirectedGraph {
  vector<Edge*> edges;
  map<string, Vertex*> vertices;
  vector<Clique> cliques;
  Clique maxCl;
public:
  Vertex *  addVertex(string label, float weight){
    Vertex * v = new Vertex(label, weight);
    vertices[label]=v;
    return v;
  }

  map<string, Vertex*> getVertices(){return vertices;}
  vector<Edge*> getEdges(){return edges;}

  Edge * addEdge(string from, string to){

    if (vertices.find(from) != vertices.end() && vertices.find(to) != vertices.end()){
      Vertex * vfrom = vertices.find(from)->second;
      Vertex * vto = vertices.find(to)->second;
      Edge * e = new Edge(vfrom,vto);
      (*vfrom).addEdge(e);
      (*vto).addEdge(e);
      edges.push_back(e);
      return e;
    }
    else{
      //needt o handle case where vertices did not exist.
      return 0;
    }
  }

  Edge * getEdge(string from, string to){
    if (vertices.find(from) != vertices.end() && vertices.find(to) != vertices.end()){
      Vertex * v1 = vertices.find(from)->second;
      Vertex* v2 = vertices.find(to)->second;
      Edge * e = (*v1).getEdgeTo(to);
      return e;
    }
    else {
      //need to handle case where vertices did not exist.
      return 0;
    }
  }

  void removeEdge(string from, string to){
    Edge * e = getEdge(from,to);
    if (e != 0){
      edges.erase(remove(edges.begin(),edges.end(),e),edges.end());
      (*e).getV1()->removeEdge(e);
      (*e).getV2()->removeEdge(e);
    }
    //handle case where edge did not exist?
  }

  Vertex * getVertexWithLabel(string l){
    if (vertices.find(l) != vertices.end())
      return vertices.find(l)->second;
    else
      return 0;
  }

  void removeVertex(string l){
    Vertex * v = getVertexWithLabel(l);
    if (v != 0){
      vector<Edge *> edges = getVertexWithLabel(l)->getEdges();

      for (vector<Edge *>::iterator it = edges.begin(); it != edges.end(); ++it){
        string from = (*it)->getV1()->getLabel();
        string to = (*it)->getV2()->getLabel();
        removeEdge(from,to);
      }
      vertices.erase(l);
    }
    else {
      //Need to handle case where vertex does not exist.
     }
  }

  vector<Vertex *> whereCanIGo(Vertex * v)
  {
    vector<Vertex *> destinations;
    vector<Edge *> edges = v->getEdges();
    for (vector<Edge *>::const_iterator it = edges.begin(); it != edges.end(); ++it) {
      if ((*it)->getV1() != v){
    destinations.push_back((*it)->getV1());
      }
      if ((*it)->getV2() !=v) {
    destinations.push_back((*it)->getV2());
      }
    }      
    destinations.push_back(v);
    return destinations;
  }

  void do_partition_find_maximum_clique() {
    int verticesSize = vertices.size();
    int threadscont = 1;
    std::thread myThreads[threadscont];
    float chunkSize =  ceil(verticesSize / threadscont);

    for(int i=1; i<=threadscont; i++) {
      map<string, Vertex*> newVertices = getSubMap((i-1)*chunkSize, i*chunkSize);
      myThreads[i-1] = std::thread(&UnDirectedGraph::findCliques, this, newVertices);
    }

    for (int i=0; i<threadscont; i++){
      myThreads[i].join();
    }

    cout<<"All cliques found by all threads.."<<endl;
    int maxWeight = 0;
    Clique maxClique;
    for (Clique c: cliques) {
      int currentWeight = c.getTotalWeight();
      if(currentWeight > maxWeight ) {
        maxClique = c;
      }
    }

    cout << "Maximum CLique weight is: " << maxClique.getTotalWeight() << endl;
    cout << "Max Clique is :" << endl;
    for (string i: maxClique.getVertices()) {
      cout << i << ", ";
    }

    cout << endl;
  }

  void findCliques(map<string, Vertex*> gTemp) {
    bool found = false;
    cout << "Finding Cliques..." << endl;
    float max = 0;
    auto iter = gTemp.begin();
    int count = 0;
    while (iter != gTemp.end()) {
      Vertex * v = iter->second;
      string label = iter->first;
      vector<Edge *> edges = v->getEdges();

      for(Edge * e : edges) {
        Clique c = Clique();

        c.addVertex(label);
        c.updateWeight(v->getWeight());
        Vertex * v1 = e->getV1();
        if(v1->getLabel()!= label) {
          c.addVertex(v1->getLabel());
          c.updateWeight(v1->getWeight());
          cliques.push_back(c);

          vector<Edge *> neighbours = v1->getEdges();
          for(Edge * n : neighbours) {
            Vertex * g1 = n->getV1();
            Vertex * g2 = n->getV2();
            if(g1->getLabel()!=v1->getLabel() && g1->getLabel()!=label) {
              if(g1->getEdgeTo(label)!=0) {
                // Vertex * temp1 = this->getVertexWithLabel(g1->getLabel());
                c.addVertex(g1->getLabel());
                c.updateWeight(g1->getWeight());
                cliques.push_back(c);
              }
            }

            if(g2->getLabel()!=v1->getLabel() && g2->getLabel()!=v->getLabel()) {
              if(g2->getEdgeTo(label)!=0) {
                // Vertex * temp1 = this->getVertexWithLabel(g2->getLabel());
                c.addVertex(g2->getLabel());
                c.updateWeight(g2->getWeight());
                cliques.push_back(c);
              }
            }
          }
        } else {
          Vertex * v2 = e->getV2();
          if(v2->getLabel()!= label) {
            c.addVertex(v2->getLabel());
            c.updateWeight(v2->getWeight());

            cliques.push_back(c);

            vector<Edge *> neighbours = v->getEdges();
            for(Edge * n : neighbours) {
              Vertex * g1 = n->getV1();
              Vertex * g2 = n->getV2();
              if(g1->getLabel()!=v2->getLabel() && g1->getLabel()!=label) {
                if(g1->getEdgeTo(label)!=0) {
                  Vertex * temp1 = this->getVertexWithLabel(g1->getLabel());
                  c.addVertex(temp1->getLabel());
                  c.updateWeight(temp1->getWeight());
                  cliques.push_back(c);
                }
              }

              if(g2->getLabel()!=v2->getLabel() && g2->getLabel()!=label) {
                if(g2->getEdgeTo(label)!=0) {
                  Vertex * temp1 = this->getVertexWithLabel(g2->getLabel());
                  c.addVertex(temp1->getLabel());
                  c.updateWeight(temp1->getWeight());
                  cliques.push_back(c);
                }
              }
            }
            
          }
        }
        // cliques.push_back(c);
      }
      ++count;
      ++iter; 

      // free(v);
    }

    // auto iter2 = gTemp.begin();
    // // int count = 0;
    // while (iter2 != gTemp.end()) {
    //   Vertex * vt = iter2->second;
    //   string label = iter2->first;
    //   vector<Edge *> edges = vt->getEdges();

    //   Clique c1 = Clique();
    //   c1.addVertex(vt->getLabel());
    //   c1.updateWeight(vt->getWeight());

    //   for(Edge * e1 : edges) {
    //     Vertex * vt1 = e1->getV1();
    //     if(vt1->getLabel()!= vt->getLabel()) {
    //       if(vt1->getEdgeTo(vt->getLabel())!=0) {
    //         Vertex * temp1 = this->getVertexWithLabel(vt1->getLabel());
    //         c1.addVertex(temp1->getLabel());
    //         c1.updateWeight(temp1->getWeight());
    //         cliques.push_back(c1);
    //       }
    //     } else {
    //       Vertex * vt2 = e1->getV2();
    //       if(vt2->getLabel()!= vt->getLabel()) {
    //         if(vt2->getEdgeTo(vt->getLabel())!=0) {
    //           Vertex * temp1 = this->getVertexWithLabel(vt2->getLabel());
    //           c1.addVertex(temp1->getLabel());
    //           c1.updateWeight(temp1->getWeight());
    //           cliques.push_back(c1);
    //         }
    //       }
    //     }
    //   }
    //   // ++count;
    //   ++iter2; 
    // }
  }



  map<string, Vertex*> getSubMap(int start, int end) {
    map<string, Vertex*> newVertices;

    int verticesSize = vertices.size();
    if(end> verticesSize) {
      end = verticesSize;
    }

    auto iter = vertices.begin();
    int count = 0;
    while (iter != vertices.end()) {
        if(count>=start && count<=end) {
          Vertex * v = iter->second;
          newVertices[iter->first]=v;
        }
        ++count;
        ++iter; 
    }

    return newVertices;
  }

};


void printGraph(UnDirectedGraph * t){

  map<string,Vertex*> vertices = t->getVertices();

  for (map<string, Vertex*>::iterator it = vertices.begin(); it != vertices.end(); ++it){
    cout << it->first <<": ";
    vector<Edge *> edges = it->second->getEdges();
    for (vector<Edge *>::iterator jit = edges.begin(); jit != edges.end(); ++jit){
        string l1 = (*jit)->getV1()->getLabel();
    string l2=(*jit)->getV2()->getLabel();
    if (l1 != it->first){cout << l1 << ", ";}
    if (l2 != it->first){cout << l2 << ", ";}
    }
    cout << endl;
    }
}

bool isPath(UnDirectedGraph * t, string from, string to)
{
  Vertex * vfrom = t->getVertexWithLabel(from);
  Vertex * vto = t->getVertexWithLabel(to);

  if (vfrom == 0 || vto == 0) {
    return false;
  }

  if (from==to) {
    return true;
  }


  UnDirectedGraph g = *t;

  map<string, Vertex*> vertices = t->getVertices();
  vector<Edge *> edges = t->getEdges();

  vector<Vertex *> verticesToCheck;
  verticesToCheck.push_back(vfrom);
  vertices.erase(from);

  while (verticesToCheck.size() != 0){
    vector<Vertex *> destinations = t->whereCanIGo(verticesToCheck[0]);
      verticesToCheck.erase(verticesToCheck.begin());


      for (vector<Vertex *>::const_iterator it = destinations.begin(); it != destinations.end(); ++it) {
    //
    if (vertices.find((*it)->getLabel()) != vertices.end()) {
      if ((*it)->getLabel()==to) {
        return true;
      }
      verticesToCheck.push_back((*it));
      vertices.erase((*it)->getLabel());
    }
      }
  }
  return false;
}
