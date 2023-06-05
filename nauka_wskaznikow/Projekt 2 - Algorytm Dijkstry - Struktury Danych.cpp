#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <limits>
#include <random>
#include <chrono>
#include <string>
#include "Graf.h"

// Struktura reprezentująca wierzchołek w algorytmie Dijkstry
struct Vertex {
    int index;
    int distance;

    Vertex(int idx, int dist) : index(idx), distance(dist) {}

    // Operator porównania dla kolejki priorytetowej
    bool operator<(const Vertex& other) const {
        return distance > other.distance;
    }
};

// Funkcja implementująca algorytm Dijkstry na kolejce priorytetowej
void Dijkstra_on_pq(const Graf& graph, int source, std::vector<int>& distances);

// Funkcja implementująca algorytm Dijkstry na stosie
void Dijkstra_on_st(const Graf& graph, int source, std::vector<int>& distances);


int main() {
    // Tworzenie grafu
    int numVertices = 1200;
    Graf graph(numVertices);

    for (int i = 0; i < numVertices; i++)
    {
        for (int j = 0; j < numVertices; j++)
        {
            //Radnomizer do dlugosci krawedzi
            std::random_device rd;
            std::mt19937 gen(rd());
            int min = 1;
            int max = 100;
            std::uniform_int_distribution<int> dist(min, max);
            int randomNum = dist(gen);
            graph.addEdge(i, j, randomNum);
        };
    };
    // Odkomentuj jesli chcesz wyswietlac sasiadow poszczegolnych wierzcholkow
    /*for (int k = 0; k < numVertices; k++) {
        std::cout << "Somsiedzi wierzcholka " << k << ": ";
        const std::vector<Edge>& neighbors = graph.getNeighbors(k);
        for (const Edge& edge : neighbors) {
            std::cout << "(" << edge.destination << ", " << edge.weight << ") ";
        }
        std::cout << std::endl << std::endl;
    }*/

    int source = 1;  // Wybierz od korego wierzcholka maja byc mierzone trasy

    // Wywołanie algorytmu Dijkstry
    std::vector<int> distances;

    auto start = std::chrono::high_resolution_clock::now();

    Dijkstra_on_st(graph, source, distances);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    // Odkomentuj jesli chcesz wyswietlic najkrotsze sciezki do poszczeolnych wierzcholkow
    /*std::cout << std::endl << "Najkrotsze sciezki od wierzcholka " << source << ":" << std::endl;
    for (int i = 0; i < numVertices; ++i) {
        std::cout << "Wierzcholek " << i << ": " << distances[i] << std::endl;
    };*/

    std::cout << "Czas dzialania: " << duration << std::endl << std::endl;


    auto start2 = std::chrono::high_resolution_clock::now();

    Dijkstra_on_pq(graph, source, distances);

    auto end2 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(end2 - start2).count();

    // Wyświetlanie wyników
    /*std::cout << "Najkrotsze sciezki od wierzcholka " << source << ":" << std::endl;
    for (int i = 0; i < numVertices; ++i) {
        std::cout << "Wierzcholek " << i << ": " << distances[i] << std::endl;
    };*/

    std::cout << "Czas dzialania: " << duration2 << std::endl << std::endl;
    
    return 0;
}

void Dijkstra_on_pq(const Graf& graph, int source, std::vector<int>& distances)
{
    int numVertices = graph.getNumVertices();
    distances.assign(numVertices, std::numeric_limits<int>::max()); // Inicjalizacja dystansów jako nieskończoność
    distances[source] = 0;

    std::priority_queue<Vertex> pq;
    pq.push(Vertex(source, 0));

    while (!pq.empty()) {
        Vertex current = pq.top();
        pq.pop();

        if (current.distance > distances[current.index])
            continue;

        const std::vector<Edge>& neighbors = graph.getNeighbors(current.index);
        for (const Edge& edge : neighbors) {
            int newDistance = current.distance + edge.weight;
            if (newDistance < distances[edge.destination]) {
                distances[edge.destination] = newDistance;
                pq.push(Vertex(edge.destination, newDistance));
            }
        }
    }
}

void Dijkstra_on_st(const Graf& graph, int source, std::vector<int>& distances)
{
    int numVertices = graph.getNumVertices();
    distances.assign(numVertices, std::numeric_limits<int>::max()); // Inicjalizacja dystansów jako nieskończoność
    distances[source] = 0;

    std::stack<Vertex> st;
    st.push(Vertex(source, 0));

    while (!st.empty()) {
        Vertex current = st.top();
        st.pop();

        if (current.distance > distances[current.index])
            continue;

        const std::vector<Edge>& neighbors = graph.getNeighbors(current.index);
        for (const Edge& edge : neighbors) {
            int newDistance = current.distance + edge.weight;
            if (newDistance < distances[edge.destination]) {
                distances[edge.destination] = newDistance;
                st.push(Vertex(edge.destination, newDistance));
            }
        }
    }
}
