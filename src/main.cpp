
#include "../include/graph.h"

int enter(int n = 0);

/**
 * Generates a random graph with the given number of vertices and edges.
 */
Graph generateRandomGraph(int numVertices, int numEdges)
{
    std::srand(std::time(nullptr));       // Seed the random number generator with the current time.
    bool directed = std::rand() % 2 == 0; // Randomly decide whether the graph is directed or undirected.
    Graph graph(directed);                // Create a random graph (directed or undirected) (50/50).
    // Add vertices to the graph.
    for (int i = 0; i < numVertices; ++i)
    {
        graph.addVertex(i);
    }

    // Add random edges to the graph.
    for (int i = 1; i < numEdges; ++i)
    {
        int vertex1 = std::rand() % numVertices; // Generate a random vertex index.
        int vertex2 = std::rand() % numVertices; // Generate another random vertex index.
        int weight = std::rand() % 10 + 1;       // Generate a random weight between 1 and 10.
        if (vertex1 != vertex2)                  // Ensure that the two vertices are not the same.
        {
            graph.addEdge(vertex1, vertex2, weight);
        }
    }
    return graph;
}

int main()
{
    Graph g(false);
    print("Enter the number of vertices: ");
    int numVertices;
    numVertices = enter(0);
    for (int i = 0; i < numVertices; i++)
    {
        print("Adding vertex: "), print(i), print("\n"), print("Value: ");
        int value = enter(0);
        g.addVertex(i, value);
    }

    print("Enter the number of edges: "), print("\n");
    int numEdges;
    numEdges = enter(0);

    for (int i = 0; i < numEdges; i++)
    {
        int vertex1, vertex2, weight;

        print("Enter the first vertex: "), print("\n");
        vertex1 = enter(0);
        print("Enter the second vertex: "), print("\n");
        vertex2 = enter(numVertices);
        print("Enter the weight: "), print("\n");
        weight = enter(0);
        g.addEdge(vertex1, vertex2, weight);
    }

    Graph randomGraph = generateRandomGraph(5, 10);
    print("\n"), print("Random Graph: "), print("\n");
    for (const auto &[vertex, neighbours] : randomGraph.getVertices())
    {
        print("Vertex: "), print(vertex), print("\n");
        for (const auto &[neighbour, weight] : neighbours)
        {
            print("Neighbour: "), print(neighbour), print(" Weight: "), print(weight), print("\n");
        }
    }
    print("\n"), print("Breadth-First Traversal: "), print("\n");
    BFT(randomGraph, 0);
    print("\n"), print("Depth-First Traversal: "), print("\n");
    DFT(randomGraph, 0);
    return 0;
}

/**
 * Prompts the user to enter a number.
 */
int enter(int n)
{
    if (n != 0)
    {
        while (true)
        {
            int j = 0;
            std::cin >> j;
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            if (std::cin.fail())
            {
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                print("Please enter a valid number: ");
            }
            else if (j < 0)
            {
                print("Please enter a positive number: ");
            }
            else if (j == n)
            {
                print("Please enter a number less than "), print(n), print(": ");
            }
            else if (j >= n)
            {
                print("Please enter a number less than "), print(n), print(": ");
            }
            else
            {
                n = j;
                break;
            }
        }
    }
    else
    {
        while (true)
        {
            std::cin >> n;
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            if (std::cin.fail())
            {
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                print("Please enter a valid number: ");
            }
            else if (n < 0)
            {
                print("Please enter a positive number: ");
            }
            else
            {
                break;
            }
        }
    }
    return n;
}
