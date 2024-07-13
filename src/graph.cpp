#include "../include/graph.h"

/**
 *  Prints the given value.
 */

Graph::Graph(bool directed) : directed(directed), edge_count(0) {}

/**
 * Returns whether the graph is empty.
 */
bool Graph::isEmpty() const
{
    return vertices.empty();
}

/**
 * Returns whether the graph is directed.
 */
bool Graph::isDirected() const
{
    return directed;
}

/**
 * Adds a vertex to the graph.
 */
void Graph::addVertex(int newVertex, int value)
{
    vertices[newVertex] = std::unordered_map<int, int>();
    vertices[newVertex][0] = value;
}

/**
 * Adds an edge between the given vertices with the given weight.
 */
void Graph::addEdge(int vertex1, int vertex2, int weight)
{
    vertices[vertex1][vertex2] = weight;
    if (!directed)
    {
        vertices[vertex2][vertex1] = weight;
    }
    edge_count++;
}

/**
 * Removes the given vertex from the graph.
 */
void Graph::removeVertex(int vertexToRemove)
{
    for (auto &[vertex, neighbours] : vertices)
    {
        if (neighbours.erase(vertexToRemove) > 0)
        {
            edge_count--;
        }
    }
    edge_count -= vertices[vertexToRemove].size();
    vertices.erase(vertexToRemove);
}

/**
 * Removes the edge between the given vertices.
 */
void Graph::removeEdge(int vertex1, int vertex2)
{
    if (vertices[vertex1].erase(vertex2) > 0)
    {
        edge_count--;
    }
    if (!directed && vertices[vertex2].erase(vertex1) > 0)
    {
        edge_count--;
    }
}

/**
 * Returns the number of vertices in the graph.
 */
int Graph::numVertices() const
{
    return vertices.size();
}

/**
 * Returns the number of edges in the graph.
 */
int Graph::numEdges() const
{
    return edge_count;
}

/**
 * Returns the indegree of the given vertex.
 */
int Graph::indegree(int vertex) const
{
    int indeg = 0;
    for (const auto &[v, neighbours] : vertices)
    {
        if (neighbours.count(vertex) > 0)
        {
            indeg++;
        }
    }
    return indeg;
}

/**
 * Returns the outdegree of the given vertex.
 */
int Graph::outdegree(int vertex) const
{
    return vertices.at(vertex).size();
}

/**
 * Returns the degree of the given vertex.
 */
int Graph::degree(int vertex) const
{
    if (directed)
    {
        return indegree(vertex) + outdegree(vertex);
    }
    return outdegree(vertex);
}

/**
 * Returns the neighbours of the given vertex.
 */
std::vector<int> Graph::neighbours(int vertex) const
{
    std::vector<int> result;
    for (const auto &[neighbour, weight] : vertices.at(vertex))
    {
        result.push_back(neighbour);
    }
    return result;
}

/**
 * Checks if the given vertices are neighbours.
 */
bool Graph::neighbour(int vertex1, int vertex2) const
{
    return vertices.at(vertex1).count(vertex2) > 0;
}

/**
 * Performs a breadth-first traversal of the graph starting from the given vertex.
 */
void BFT(const Graph &graph, int start, int end)
{
    if (graph.isEmpty())
    {
        print("Graph is empty.");
        return;
    }
    std::queue<int> q;
    std::unordered_set<int> visited;
    q.push(start);
    visited.insert(start);
    while (!q.empty())
    {
        int current = q.front();
        q.pop();
        print(current);
        for (int neighbour : graph.neighbours(current))
        {
            if (visited.insert(neighbour).second)
            {
                q.push(neighbour);
            }
        }
    }
}

/**
 * Performs a depth-first traversal of the graph starting from the given vertex.
 */
void DFT(const Graph &graph, int start, int end)
{
    if (graph.isEmpty())
    {
        print("Graph is empty.");
        return;
    }
    std::stack<int> s;
    std::unordered_set<int> visited;
    s.push(start);
    visited.insert(start);
    while (!s.empty())
    {
        int current = s.top();
        s.pop();
        print(current);
        for (int neighbour : graph.neighbours(current))
        {
            if (visited.insert(neighbour).second)
            {
                s.push(neighbour);
            }
        }
    }
}

/**
 * Calculates the minimum spanning tree of the graph starting from the given vertex.
 *
 * @param start The starting vertex for calculating the minimum spanning tree.
 * @return An unordered map representing the minimum spanning tree, where the key is the vertex and the value is the weight of the edge connecting it to the tree.
 */
std::unordered_map<int, int> Graph::min_spanning_tree(int start) const
{
    std::unordered_map<int, int> mst; // Renamed result to mst for clarity
    std::unordered_set<int> visited;
    std::priority_queue<std::pair<int, std::pair<int, int>>, std::vector<std::pair<int, std::pair<int, int>>>, std::greater<>> pq; // Using priority queue to automatically sort edges by weight

    // Initialize by adding all edges from the start vertex to the priority queue
    for (const auto &[neighbour, weight] : vertices.at(start))
    {
        pq.push({weight, {start, neighbour}});
    }

    visited.insert(start); // Mark the start vertex as visited

    while (!pq.empty() && visited.size() < vertices.size())
    {
        auto [weight, edge] = pq.top();
        pq.pop();
        auto [u, v] = edge;

        // If v is not visited, it's a valid edge in the MST
        if (visited.count(v) == 0)
        {
            visited.insert(v); // Mark v as visited
            mst[v] = weight;   // Add edge to the MST

            // Add all edges from the newly visited vertex to the priority queue
            for (const auto &[neighbour, nextWeight] : vertices.at(v))
            {
                if (visited.count(neighbour) == 0)
                {
                    pq.push({nextWeight, {v, neighbour}});
                }
            }
        }
    }

    return mst; // Return the MST
}