#include "Graf.h"

void Graf::addEdge(int source, int destination, int weight)
{
    adjacencyList[source].push_back(Edge(destination, weight));
    adjacencyList[destination].push_back(Edge(source, weight)); // Dla grafu nieskierowanego
}

const std::vector<Edge>& Graf::getNeighbors(int vertex) const
{
    return adjacencyList[vertex];
}

int Graf::getNumVertices() const
{
    return numVertices;
}
