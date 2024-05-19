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

#include<iostream>
#include<fstream>
#include<cstring>
#include<time.h>
#include <sys/time.h>
#include<vector>
#include<sstream>
#include<thread>
#include <cassert>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <omp.h>
#include <unistd.h>
#include <mutex>
#include <unordered_map>
#include <dirent.h>
#include <chrono>
//#include "maximum_clique.h"

using namespace std;

int ** graphMatrix;
int ** csrSparseMatrix;
int csrColSize = 0;
int * vDegree;
int maxDegree = 0;
int countNodes = 0;
int countConnections = 0;
int * vertexWeights;
float maxWeight = 0;
int * maxClique;
int maxCliqueSize = 0;
float * weights;
int * candidateListInitial;
const auto processor_count = std::thread::hardware_concurrency();
static std::mutex mtx;
bool show_log = false;
int * nodeLabels;
int avgDegree = 0;


string nodesPath = "./datasets/nodes/";
string connectionsPath = "./datasets/connections/";


unsigned long get_nano_time()
{
    struct timespec ts;
    timespec_get(&ts,TIME_UTC);
    return ts.tv_sec*1e9+ts.tv_nsec;
}


class IntersectHashFunction
{
public:
    size_t operator()(int i) const
    {
        return i;
    }
};

struct CSR {
    int rowPtrSize;
    int columnIDSize;
    int * rowPtr;
    int * columnIDs;
};
CSR csr;

struct node
{
  int label;
  float weight;
};
vector<node> nodes;

char * convertStringToChars(string str) {
    int n = str.length();

    static char * char_array = new char[n + 1];

    strcpy(char_array, str.c_str());

    return char_array;
}

void createMatrix(string filename) {
    ifstream weightFile(nodesPath+filename);

    string line;
    cout<<"----------------------"<<endl;
    cout << "Adding Vertices " << nodesPath + filename << endl;
    cout<<"----------------------"<<endl;

    while (getline(weightFile, line))
    {
        std::istringstream iss(line);
        string a;
        string b;
        if (!(iss >> a >> b)) { break; } // error

        if(a=="v") {
            countNodes = stoi(b);
            weights = new float[countNodes];
            candidateListInitial = new int[countNodes];
            nodeLabels = new int[countNodes];
        } else {

            // cout << convertStringToChars(a) << "  " << convertStringToChars(b) << endl;
            int label = stoi(a);
            
            float weight = std::stof(b);
            // cout<<"Weight: "<<weight<<endl;
            nodeLabels[label-1] = label-1;
            weights[label - 1] = weight;
            candidateListInitial[label-1] = label-1;
        }

    }

    graphMatrix = new int *[countNodes];
    vDegree = new int[countNodes]();
    maxClique = new int[countNodes]();

    for (int i = 0; i < countNodes; ++i)
    {
        graphMatrix[ i ]  = new int[countNodes];
    }

    for( int i = 0; i < countNodes; ++i )
    {
        for( int j = 0; j < countNodes; ++j )
        {
            graphMatrix[ i ][ j ] = 0;
            graphMatrix[j][i] = 0;
        }
    }
}

int findVertexWeight(int label) {
    for(node n : nodes) {
        if(n.label==label) {
            return n.weight;
        }
    }
    return 0;
}

bool compareWeights(int a, int b){
    return weights[a] > weights[b];
}


bool compareDegrees(int a, int b) {
    return vDegree[a] > vDegree[b];
}

bool compareLabels(int a, int b){
    return a < b;
}

void csrSparsify() {
    
    for (int i = 0; i < countNodes; i++) {
        for (int j = 0; j < countNodes; j++) {
            if (graphMatrix[i][j] != 0) {
                csrColSize++;
            }
        }
    }

    csr.columnIDSize = csrColSize;
    csr.columnIDs = new int[csrColSize];
    csr.rowPtrSize = countNodes+1;
    csr.rowPtr = new int[countNodes+1];

    int k = 0;
    for (int i = 0; i < countNodes; i++) {
        csr.rowPtr[i] = k;
        for (int j = 0; j < countNodes; j++) {
            if (graphMatrix[i][j] > 0 || graphMatrix[j][i] > 0)
            {
                csr.columnIDs[k] = j;
                k++;
            }
        }
    }

    csr.rowPtr[countNodes] = k;

    // To store the degree
    int totalDegree = 0;
    for (int i = 0; i < csr.rowPtrSize - 1; i++)
    {
        int from = csr.rowPtr[i];
        int to = csr.rowPtr[i + 1];
        int size = to - from;
        vDegree[i] = size;

        totalDegree+=size;

        if(size>maxDegree) {
            maxDegree = size;
        }
    }

    avgDegree = totalDegree/countNodes;
    cout<<"avgDegree: "<<avgDegree<<endl;
    // To sort the neighbours as per degree in descending order
    for (int i = 0; i < csr.rowPtrSize - 1; i++)
    {
        int from = csr.rowPtr[i];
        int to = csr.rowPtr[i + 1];

        std::sort(&csr.columnIDs[from], &csr.columnIDs[to], compareDegrees);
    }
    // cout << "----------------------" << endl;
    // cout << "Degrees" << endl;
    // cout << "----------------------" << endl;
    // for (int i = 0; i < countNodes; i++)
    // {
    //     cout << vDegree[i] << " ";
    // }
    // cout << endl;
    
    // Sort nodes 
    // std::sort(&nodeLabels[0], &nodeLabels[countNodes], compareDegrees);
}

void findNeighbours(int label) {
    int from = csr.rowPtr[label];
    int to = csr.rowPtr[label + 1];

    int j = 0;
    cout << "Neighbours for " << (label + 1) << " -> ";
    for(int i=from; i<to; i++) {
        cout << csr.columnIDs[i] + 1 << " ";
    }
    cout<<endl;
}

void generateAndDisplayAllNeighbours() {
    for(int i=0; i<csr.rowPtrSize-1; i++) {
        findNeighbours(i);
    }
}

void add_edge(int u, int v) {

    u--;
    v--;
    // cout << "adding Edge " << u + 1 << " ->  " << v + 1 << endl;
    // if(u>v)
    // {
    assert (u<countNodes && u>=0 && v>=0 && v<countNodes);

    if(u<countNodes && v<countNodes) {
        graphMatrix[ u ][ v ] = 1;
        graphMatrix[ v ][ u ] = 1;
    }
// }
}

void addEdges(string filename) {
    std::string line;
    ifstream connectionsFile(connectionsPath+filename);
    cout<<"----------------------"<<endl;
    cout << "Adding Edges" << connectionsPath + filename << endl;
    cout<<"----------------------"<<endl;

    while (getline(connectionsFile, line))
    {   
        std::istringstream iss(line);
        std::string node1;
        std::string node2;

        if (!(iss >> node1 >> node2)) { break; } // error

        if(node1=="e") {

        } else {
             //cout << stoi(node1) << "  " << stoi(node2) << endl;
            
            countConnections++;
            // if(stoi(node1)>stoi(node2))
            // {
            //cout << stoi(node1) << "  " << stoi(node2) << endl;
            add_edge(stoi(node1),stoi(node2));
           // }
        }

    }
}

void displayMatrix(int v) {
   int i, j;
   for(i = 0; i < v; i++) {
      for(j = 0; j < v; j++) {
         cout << graphMatrix[i][j] << " ";
      }
      cout << endl;
   }
}

void displayCSRSparseMatrix() {

    cout<<"Row Pointer:";
    for (int j=0; j<csr.rowPtrSize; j++) {
        cout <<" "<< csr.rowPtr[j]+1;
    }
    cout<<endl;
    cout<<"Column IDs:";
    for (int j=0; j<csr.columnIDSize; j++) {
        cout <<" "<< csr.columnIDs[j]+1;
    }
    cout<<endl;
}

void printArray(int *arr, int size) {
    for(int i=0; i<size; i++) {
        cout << arr[i]+1 << " ";
    }
    cout << endl;
}

void printStore(int * store, int storeSize)
{
    for (int i = 0; i < storeSize; i++)
    {
        cout << store[i]+1 << " | ";
    }
    cout << endl;
}

// Function to compare and replace the clique if bigger
void compareAndReplaceClique(int *store, int n)
{
    
    float localWeight = 0;
    int localCliqueSize = 0;

    for (int i = 0; i < n; i++) {
        localWeight = localWeight + weights[store[i]];
        localCliqueSize++;
    }

    mtx.lock();
    if (localWeight > maxWeight) {
        maxWeight = localWeight;
        maxCliqueSize = localCliqueSize;
        std::memcpy(maxClique, store, sizeof(int) * countNodes);
        cout << "maxWeight: " << maxWeight << endl;
    }
    mtx.unlock();
    if (show_log) {
        cout<<"Store:" << endl;
        printStore(store, n);
        cout<<"maxWeight: "<<maxWeight<<endl;
        cout<<"localWeight: "<<localWeight<<endl;
    }
}

bool is_clique(int* store, int b)
{
    // printStore(store, b);
    for (int i = 0; i < b; i++)
    {
        for (int j = i + 1; j < b; j++)
            if (graphMatrix[store[i]][store[j]] == 0)
                return false;
    }
    return true;
}

int exists(int * storeCurrent, int s, int toFind) {
    for(int i=0; i<s; i++) {
        if(storeCurrent[i]==toFind) {
            return true;
        }
    }
    return false;
}

// haven't removed commented code as it is still in debigging
void findCliqueNew(int i, int *store, int storesize, int *visited, int *candidateList, int candidateListSize, int threads)
{
    // check if the node is already in trace back array
    // if (visited[i] == 1 && candidateListSize < 1) {
    //     visited[i] = 0;
    // }
    // bool nodeExists = exists(store, storesize, i);
    // store[storesize] = i;
    // storesize++;
    // compareAndReplaceClique(store, storesize);
    // if(visited[i]==1) {
    //     return;
    // }
    if (visited[i] <= storesize*2)
    {
        // cout<< "ima here "<<nodeExists<<endl;
        // store in the trace back array called store
        store[storesize] = i;

        // if (is_clique(store, storesize + 1))
        // {
    //     // if is clique declare visited
       visited[i]++;
        // increase the count for the store for the next index
        storesize++;

        // store and replace clique if bigger and print for debugging if needed
        compareAndReplaceClique(store, storesize);

        if (candidateListSize > 0)
        {
            // get neighbours already sorted per degree from the CSR
            int min = csr.rowPtr[i];
            int max = csr.rowPtr[i + 1];

            // cout << "Candidate List" << endl;
            // printArray(candidateList, candidateListSize);

            // Find intersection between neighbours and current candidate list
            int intersection[max - min];

            int m = 0;
            unordered_map<int, int, IntersectHashFunction> candidateHash;
            for (int k = 0; k < candidateListSize; k++)
            {
                candidateHash[candidateList[k]] = 1;
            }
            for (int n = min; n < max; n++)
            {
                if (candidateHash.find(csr.columnIDs[n]) == candidateHash.end())
                {
                }
                else
                {
                    intersection[m] = csr.columnIDs[n];
                    m++;
                }
            }

            if (show_log)
            {
                cout << "Candidate List (Intersection)" << endl;
                printArray(intersection, m);
            }

            // add nodes from candidatelist check if is clique
            if (m > 0)
            {
		        // std::sort(&intersection[0], &intersection[m], compareDegrees);
                // cout << "Candidate List (Intersection)" << endl;
                // printArray(intersection, m);
                omp_set_num_threads(threads);
                #pragma omp parallel for
                for (int z = 0; z < m; z++)
                {
                    if (show_log)
                    {
                        cout << "Going from: " << i + 1 << " to : " << intersection[z] + 1 << " store Size: " << storesize << endl;
                    }
                    // int visited1[countNodes];
                    // std::fill(visited1, visited1 + countNodes, 0);
                    // if (visited[i] == 0)
                    findCliqueNew(intersection[z], store, storesize, visited, intersection, m, threads);
                    // visited[intersection[z]] = 1;
                }

                // delete[] intersection;
            } else {
                return;
            }
        }
        // }
    }
    // else
    // {
    //     visited[i] = 0;
    // }
}

void resetVisited(int* visited) {
    for(int i=0; i<countNodes; i++) {
        visited[i] = 0;
    }
}

void findMaximumClique(int threads) {
    // To set total number of the threads taken from processor cores automatically but can be set manually
    std::sort(&nodeLabels[0], &nodeLabels[countNodes], compareDegrees);
    show_log = false;
    omp_set_num_threads(threads);
   #pragma omp parallel for 
   //#pragma omp parallel for schedule(dynamic,32)
    for (int j = 0; j < countNodes; j++)
    {
        int store[countNodes];
        int visited[countNodes];
        std::fill(visited, visited + countNodes, 0);

        int min = csr.rowPtr[j];
        int max = csr.rowPtr[j + 1];
        int candidateListInitialOptimised[max - min];
        for (int n = min; n < max; n++){
            candidateListInitialOptimised[n-min] = csr.columnIDs[n];
        }

        if (show_log) {

            cout << "-----------------------------------------" << endl;    
            cout << "Starting Now from: " << j + 1 << " store Size: " << 0 << endl;
            cout<<"Candidate List"<<endl;
            printArray(candidateListInitialOptimised, countNodes);
        }

        // Find cliques and replace with greater one
        findCliqueNew(nodeLabels[j], store, 0, visited, candidateListInitialOptimised, max-min, threads);
        // delete[] store;
        // delete[] visited;
        // break;
    }
}



int main(int argc, char *argv[])
{
    cout << "----------------------------------" << endl;
    cout << "Starting first round .." << endl;
    cout << "----------------------------------" << endl;

    ofstream resultsCsv;
    resultsCsv.open("output.csv", std::ios::app);
    resultsCsv << "Graph,Number of Nodes,Number of connections,Max Weight,Number of Nodes in Clique,Nodes of Clique,Time Taken\n";
    resultsCsv.close();
    DIR *dr;
    dirent *en;
    dr = opendir("./datasets/nodes"); // open all or present directory

    if (dr)
    {
        while ((en = readdir(dr)) != NULL)
        {
            countNodes = 0;
            countConnections = 0;
            maxWeight = 0;
           // delete []maxClique;
            maxCliqueSize = 0;

            

            if (strcmp(en->d_name, ".") && strcmp(en->d_name, "..")){
                string fileName = en->d_name;
                if (fileName != "")
                // if (fileName == "test2.txt") // C125-9.txt
                {
                    // MaximumClique mc("./dataset/nodes/" + string(en->d_name), "./1-dataset/connections/" + string(en->d_name), 10, false);

                    // maxWeight = mc.getMaxCliqueWeight();
                    // maxCliqueSize = mc.getMaxCliqueSize();
                    // maxClique = mc.getMaxClique();

                    // int* visited = mc.getVisited();
                    // int * visited = mc.getVisited();

                    cout << "---------------------------" << endl;
                    cout<< "Checking for file : "<<en->d_name<<endl; // print all directory name
                    cout << "---------------------------" << endl;

                    int threads = 1;
                    if (argv[1] == string("-p"))
                    {
                        threads = atoi(argv[2]);
                        cout<<"Threads: "<<threads<<endl;
                    }
              
                    cout << "---------------------------" << endl;
                    cout << "Maximum Weighted Clique Problem..." << endl;
                    cout << "---------------------------" << endl;

                    createMatrix(en->d_name);
                    addEdges(en->d_name);

                    // cout << "---------------------------" << endl;
                    // cout << "Adjacency Matrix" << endl;
                    // cout << "---------------------------" << endl;

                    // displayMatrix(countNodes);
                    csrSparsify();
                    // cout << "--------------------------------------" << endl;
                    // cout << "Neighbours using CSR (Sorted by Weights)" << endl;
                    // cout << "--------------------------------------" << endl;
                    // generateAndDisplayAllNeighbours();

                    // cout << "---------------------------" << endl;
                    // cout << "CSR of Sparse Matrix" << endl;
                    // cout << "---------------------------" << endl;

                    // displayCSRSparseMatrix();
                    cout << "---------------------------" << endl;
                    cout << "Finding Cliques" << endl;
                    cout << "---------------------------" << endl;
                        // clock_t tStart = clock();
                    
                    unsigned long timeStart = get_nano_time();

                    double powerPerThread = 10.0; // Power consumption per thread (in watts)
                    double executionTime = 0.0;   // Total execution time (in seconds)
                    auto startTime = std::chrono::high_resolution_clock::now();

                    findMaximumClique(threads);

                    auto endTime = std::chrono::high_resolution_clock::now();

                    // Calculate execution time
                    std::chrono::duration<double> duration = endTime - startTime;
                    executionTime = duration.count();

                    // Calculate energy consumption
                    double energyConsumed = powerPerThread * threads * executionTime;

                    std::cout << "Energy consumed: " << energyConsumed << " Joules" << std::endl;
                    double ExecutionTime = (get_nano_time() - timeStart) / 1e9;
                    
                    cout << "---------------------------" << endl;
                    cout << endl;

                    cout << "Max Clique Size: " << maxCliqueSize << endl;
                    cout << "Max Weight: " << maxWeight << endl;
                    cout << "Max Clique: ";
                    string maxCliqueStr = "";
                    for (int i=0; i<maxCliqueSize; i++)
                    {
                        maxCliqueStr = maxCliqueStr+to_string(maxClique[i]+1)+" ";
                        cout<< maxClique[i] + 1 << " ";
                    }
                    cout<<endl;
                    // bool verifyClique = is_clique(maxClique, maxCliqueSize);
                    // if(verifyClique) {
                    //     cout << "Verification using adjacency matrix -> Max Clique is a TRUE Clique" << endl;
                    // } else {
                    //     cout << "Verification using adjacency matrix -> Max Clique is NOT a Clique" << endl;
                    // }

                    //double timeTaken = (double)(clock() - tStart) / CLOCKS_PER_SEC;
                     
                   // cout << "Time taken: " << timeTaken << " Seconds" << endl;
                     cout << "Execution time: " << ExecutionTime << " Seconds" << endl;

                     resultsCsv.open("output.csv", std::ios::app);
                     resultsCsv << fileName + "," + to_string(countNodes) + "," + to_string(countConnections) + "," + to_string(maxWeight) + "," + to_string(maxCliqueSize) + "," + maxCliqueStr + "," + to_string(ExecutionTime) + " Sec\n ";
                     resultsCsv.close();

                     cout << "Max Degree: " << maxDegree << endl;
                }
            }
        }
        closedir(dr); // close all directory
        
    }

    return 0;
}
