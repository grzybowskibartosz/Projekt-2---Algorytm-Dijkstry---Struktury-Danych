#include <vector>

// Struktura reprezentuj�ca kraw�d� w grafie
struct Edge {
    int destination;
    int weight;

    Edge(int dest, int w) : destination(dest), weight(w) {}
};

// Klasa reprezentuj�ca graf z wykorzystaniem listy s�siedztwa
class Graf {
private:
    int numVertices;
    std::vector<std::vector<Edge>> adjacencyList;

public:
    // Konstruktor
    Graf(int numVertices) : numVertices(numVertices) { adjacencyList.resize(numVertices); };

    // Funkcja dodaj�ca kraw�d� do grafu
    void addEdge(int source, int destination, int weight);

    // Funkcja zwracaj�ca list� s�siedztwa danego wierzcho�ka
    const std::vector<Edge>& getNeighbors(int vertex) const;

    // Funkcja zwracaj�ca liczb� wierzcho�k�w w grafie
    int getNumVertices() const;
};