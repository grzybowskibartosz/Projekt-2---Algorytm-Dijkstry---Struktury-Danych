#include <vector>

// Struktura reprezentuj¹ca krawêdŸ w grafie
struct Edge {
    int destination;
    int weight;

    Edge(int dest, int w) : destination(dest), weight(w) {}
};

// Klasa reprezentuj¹ca graf z wykorzystaniem listy s¹siedztwa
class Graf {
private:
    int numVertices;
    std::vector<std::vector<Edge>> adjacencyList;

public:
    // Konstruktor
    Graf(int numVertices) : numVertices(numVertices) { adjacencyList.resize(numVertices); };

    // Funkcja dodaj¹ca krawêdŸ do grafu
    void addEdge(int source, int destination, int weight);

    // Funkcja zwracaj¹ca listê s¹siedztwa danego wierzcho³ka
    const std::vector<Edge>& getNeighbors(int vertex) const;

    // Funkcja zwracaj¹ca liczbê wierzcho³ków w grafie
    int getNumVertices() const;
};