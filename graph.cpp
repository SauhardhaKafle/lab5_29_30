#include <iostream>
#include <vector>

class Graph
{
private:
    std::vector<std::vector<int>> adj_list;
    bool directed;

public:
    Graph(int n, bool is_directed = false) : adj_list(n), directed(is_directed){};

    bool isEmpty();
    bool isDirected();
    void addVertex(int v);
    void addEdge(int u, int v);
    void removeEdge(int u, int v);
    bool hasEdge(int u, int v) const;
    std::vector<int> getNeighbors(int v) const;
    int numVertex() const;
    int numEdge() const;
    int degree(int v) const;
    void print() const;
    void removeDirectedEdge(int u, int v);

    void removeVertex(int vertexToRemove);
    int indegree(int vertex) const;
    int outdegree(int vertex) const;
    bool neighbour(int vertex1, int vertex2) const;
};

bool Graph::isEmpty()
{
    return adj_list.empty();
};

bool Graph::isDirected()
{
    return directed;
};

void Graph::addVertex(int v)
{
    adj_list.push_back(std::vector<int>(v));
};

void Graph::addEdge(int u, int v)
{
    // Check if the edge already exists
    for (int i = 0; i < adj_list[u].size(); ++i)
    {
        if (adj_list[u][i] == v)
            return; // Edge already exists
    }
    adj_list[u].push_back(v);
    if (!directed)
    {
        adj_list[v].push_back(u);
    }
};

void Graph::removeEdge(int u, int v)
{
    removeDirectedEdge(u, v);
    if (!directed)
    {
        removeDirectedEdge(v, u);
    }
};

bool Graph::hasEdge(int u, int v) const
{
    for (int i = 0; i < adj_list[u].size(); ++i)
    {
        if (i == v)
            return true;
    }
    return false;
};

std::vector<int> Graph::getNeighbors(int v) const
{
    return adj_list[v];
};

int Graph::numVertex() const
{
    return adj_list.size();
};

int Graph::numEdge() const
{
    int count = 0;
    for (const auto &neighbors : adj_list)
    {
        count += neighbors.size();
    }
    return directed ? count : count / 2;
};

int Graph::degree(int v) const
{
    return adj_list[v].size();
};

void Graph::print() const
{
    for (int i = 0; i < adj_list.size(); ++i)
    {
        std::cout << i << ": ";
        for (int neighbor : adj_list[i])
        {
            std::cout << neighbor << " ";
        }
        std::cout << std::endl;
    }
};

void Graph::removeDirectedEdge(int u, int v)
{
    std::vector<int> &neighbors = adj_list[u];
    for (int i = 0; i < neighbors.size(); ++i)
    {
        if (neighbors[i] == v)
        {
            // Swap with the last element and pop
            neighbors[i] = neighbors.back();
            neighbors.pop_back();
            break;
        }
    }
};

void Graph::removeVertex(int vertexToRemove)
{
    if (vertexToRemove < 0 || vertexToRemove >= adj_list.size())
        return;

    // Remove the vertex from the adjacency list
    adj_list.erase(adj_list.begin() + vertexToRemove);

    // Remove all edges pointing to this vertex
    for (auto &neighbors : adj_list)
    {
        for (int i = 0; i < neighbors.size(); ++i)
        {
            if (neighbors[i] == vertexToRemove)
            {
                neighbors.erase(neighbors.begin() + i);
                --i;
            }
            else if (neighbors[i] > vertexToRemove)
            {
                --neighbors[i];
            }
        }
    }
}

int Graph::indegree(int vertex) const
{
    if (vertex < 0 || vertex >= adj_list.size())
        return 0;

    int count = 0;
    for (const auto &neighbors : adj_list)
    {
        for (int neighbor : neighbors)
        {
            if (neighbor == vertex)
                ++count;
        }
    }
    return count;
}

int Graph::outdegree(int vertex) const
{
    if (vertex < 0 || vertex >= adj_list.size())
        return 0;

    return adj_list[vertex].size();
}

bool Graph::neighbour(int vertex1, int vertex2) const
{
    if (vertex1 < 0 || vertex1 >= adj_list.size() || vertex2 < 0 || vertex2 >= adj_list.size())
        return false;

    for (int neighbor : adj_list[vertex1])
    {
        if (neighbor == vertex2)
            return true;
    }
    return false;
}

int main()
{
    Graph g(4); // Create an undirected graph with 4 vertices

    g.addEdge(0, 1);
    g.addEdge(0, 2);
    g.addEdge(1, 2);
    g.addEdge(2, 3);

    std::cout << "Graph representation:" << std::endl;
    g.print();

    std::cout << "is empty : " << (g.isEmpty() ? "True" : "False") << std::endl;
    std::cout << "is directed : " << (g.isDirected() ? "True" : "False") << std::endl;

    std::cout << "Neighbors of vertex 2: ";
    for (int neighbor : g.getNeighbors(2))
    {
        std::cout << neighbor << " ";
    }
    std::cout << std::endl;

    std::cout << "Edge count: " << g.numEdge() << std::endl;
    std::cout << "Degree of vertex 2: " << g.degree(2) << std::endl;

    std::cout << "Removing edge (1, 2)" << std::endl;
    g.removeEdge(1, 2);
    g.print();

    std::cout << "Has edge (0, 2): " << (g.hasEdge(0, 2) ? "Yes" : "No") << std::endl;
    std::cout << "Has edge (1, 2): " << (g.hasEdge(1, 2) ? "Yes" : "No") << std::endl;

    return 0;
};

/*
1. Implement graph data structure with the following operations:
//(a) isEmpty(): Returns true if the graph is empty, and false otherwise
//(b) isDirected(): Returns true if the graph is directed, and false otherwise
//(c) addVertex(newVertex): Inserts a new vertex to the graph
//(d) addEdge(vertex1, vertex2): Adds an edge from vertex1 to vertex2
//(k) degree(vertex): Returns the degree of a vertex
//(f) removeEdge(vertex1, vertex2): Remove an edge from the graph
//(g) numVertices(): Returns the number of vertices in the graph
//(h) numEdges(): Returns the number of edges in the graph
//(l) neighbours(vertex): Returns the neighbours of a vertex

//(e) removeVertex(vertexToRemove): Remove a vertex from the graph
(i) indegree(vertex): Returns the indegree of a vertex
(j) outdegree(vertex): Returns the outdegree of a vertex
//(m) neighbour(vertex1, vertex2): Returns true if vertex2 is a neighbour of vertex1.
*/