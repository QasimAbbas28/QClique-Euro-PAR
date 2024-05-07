#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <string>
#include <algorithm>
#include <thread>
#include <cmath>

using namespace std;
class Vertex;
// class Edge;

class Edge {
    Vertex * vertex1;
    Vertex * vertex2;
    public:
    Vertex getV1() const {return &vertex1;}
    Vertex getV2() const {return &vertex2;}
    
    void setV1(Vertex v){vertex1=&v;}
    void setV2(Vertex v){vertex2=&v;}

    Edge(Vertex v1, Vertex v2){
        vertex1=&v1;
        vertex2=&v2;
    }
};

class Vertex {
    int label;
    float weight;
    vector<Edge> neighbours;
    bool visited;

    public:
    int getLabel() const {
        return label;
    }

    float getWeight() const {
        return weight;
    }

    vector<Edge> getEdges()const{
        return neighbours;
    }

    void setWeight(float w){
        weight=w;
    }

    // Edge getEdgeTo(int d){
    //   for (vector<Edge>::iterator it = neighbours.begin(); it != neighbours.end(); ++it){
    //     if ((it)->getV2()->getLabel()==d){
    //       return (it);
    //     }
    //   }
    //   return null;
    // }

    void setVisited(bool v){visited=v;}
    bool getVisited() {return visited;}
    void addEdge(Edge e){neighbours.push_back(e);}
    void removeEdge(Edge e){
      neighbours.erase(remove(neighbours.begin(),neighbours.end(),e),neighbours.end());
    }

    void removeEdgeTo(int l){
      Edge e = getEdgeTo(l);
      removeEdge(e);
    }

    int getDegree() {
      return neighbours.size();
    }

    Vertex(int l, float w){
      label=l; 
      visited=false;
      weight=w;
    }

};