#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <string>
#include <algorithm>

using namespace std;

class Graph;
class Edge;
class Vertex;

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
  vector<Edge *> edgesLeavingMe;
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

  Vertex(string l, float w){
    label=l; 
    visited=false;
    weight=w;
  }
};

class Graph {
  vector<Edge*> edges;
  map<string, Vertex*> vertices;
public:
  Vertex * addVertex(string label, float weight){
    Vertex * v = new Vertex(label, weight);
    vertices[label]=v;
    return v;
  }
  Edge * addEdge(string from, string to);
  void removeEdge(string from, string to);
  Vertex * getVertexWithlabel(string l);
  void removeVertex(string l);
};

class UnDirectedGraph {
  vector<Edge*> edges;
  map<string, Vertex*> vertices;
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

};

class DirectedGraph {
  vector<Edge*> edges;
  map<string, Vertex*> vertices;
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
      edges.push_back(e);
      return e;
    }
    else{
      //handle case where vertcies did not exist.
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
      return 0;
    }
  }


  void removeEdge(string from, string to){
    Edge * e = getEdge(from,to);
    if (e != 0){
      edges.erase(remove(edges.begin(),edges.end(),e),edges.end());
      (*e).getV1()->removeEdge(e);
    }
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
      //handle case where vertex did not exist.
    }
  }

  vector<Vertex *> whereCanIGo(Vertex * v)
  {
    vector<Vertex *> destinations;
    vector<Edge *> edges = v->getEdges();
    for (vector<Edge *>::const_iterator it = edges.begin(); it != edges.end(); ++it) {
      if ((*it)->getV2() !=v) {
    destinations.push_back((*it)->getV2());
      }
    }      
    destinations.push_back(v);
    return destinations;
  }

};



template <class T> 
void printGraph(T * t){

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

template <class T> 
bool isPath(T * t, string from, string to)
{
  Vertex * vfrom = t->getVertexWithLabel(from);
  Vertex * vto = t->getVertexWithLabel(to);

  if (vfrom == 0 || vto == 0) {
    return false;
  }

  if (from==to) {
    return true;
  }


  T g = *t;

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
