#include <iostream>
#include <vector>
#include <limits>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>

using namespace std;

const int Dmax = numeric_limits<int>::max();
const int DQmax = 1000;

class Node
{
public:
    int x, y;
    int distanceToGoal;
    bool isObstacle;
    bool isAcceleratingNode;
    Node *successor;

    Node(int x, int y) : x(x), y(y), distanceToGoal(Dmax), isObstacle(false),
                         isAcceleratingNode(false), successor(nullptr) {}

    void setObstacle(bool state) { isObstacle = state; }
    void setAcceleratingNode(bool state) { isAcceleratingNode = state; }
};

class Map
{
private:
    int width, height;
    vector<vector<Node>> grid;
    Node *start;
    Node *goal;

public:
    Map(int width, int height) : width(width), height(height), grid(width, vector<Node>(height, Node(0, 0)))
    {

        initializeGrid();
    }

    void initializeGrid()
    {
        for (int i = 0; i < width; ++i)
        {
            for (int j = 0; j < height; ++j)
            {
                grid[i][j] = Node(i, j);
            }
        }
    }

    Node *getNode(int x, int y)
    {
        if (x >= 0 && x < width && y >= 0 && y < height)
            return &grid[x][y];
        return nullptr;
    }

    void setObstacle(int x, int y)
    {
        Node *node = getNode(x, y);
        if (node)
            node->setObstacle(true);
    }

    void setStartAndGoal(int startX, int startY, int goalX, int goalY)
    {
        start = getNode(startX, startY);
        goal = getNode(goalX, goalY);
        if (goal)
            goal->distanceToGoal = 0;
    }

    Node *getStart() { return start; }
    Node *getGoal() { return goal; }
    int getWidth() { return width; }
    int getHeight() { return height; }

    vector<Node *> getNeighbors(Node *node)
    {
        vector<Node *> neighbors;
        int x = node->x, y = node->y;
        int dx[] = {-1, 1, 0, 0};
        int dy[] = {0, 0, -1, 1};
        for (int i = 0; i < 4; ++i)
        {
            Node *neighbor = getNode(x + dx[i], y + dy[i]);
            if (neighbor && !neighbor->isObstacle)
            {
                neighbors.push_back(neighbor);
            }
        }
        return neighbors;
    }
};

class AcceleratingNodes
{
private:
    Map &map;
    vector<Node *> acceleratingNodes;

public:
    AcceleratingNodes(Map &map_) : map(map_) {}

    void addAcceleratingNode(int x, int y)
    {
        Node *node = map.getNode(x, y);
        if (node)
        {
            node->setAcceleratingNode(true);
            acceleratingNodes.push_back(node);
        }
    }

    void monitorChanges()
    {
        if (detectBlockage())
        {
            cout << "Blockage detected! Switching to alternate path." << endl;
            vector<Node *> altPath = findAlternativePath(map.getStart(), map.getGoal());
            if (!altPath.empty())
            {
                printPath(altPath);
            }
            else
            {
                cout << "No alternate path found." << endl;
            }
        }
        else
        {
            cout << "No blockage detected, continuing on DP path." << endl;
        }
    }

private:
    bool detectBlockage()
    {
        Node *current = map.getStart();
        while (current != nullptr && current != map.getGoal())
        {
            if (current->isObstacle)
                return true;
            current = current->successor;
        }
        return false;
    }

    vector<Node *> findAlternativePath(Node *start, Node *goal)
    {
        priority_queue<pair<int, Node *>, vector<pair<int, Node *>>, greater<>> pq;
        unordered_map<Node *, Node *> cameFrom;
        unordered_map<Node *, int> costSoFar;

        pq.emplace(0, start);
        costSoFar[start] = 0;

        while (!pq.empty())
        {
            Node *current = pq.top().second;
            pq.pop();

            if (current == goal)
                break;

            for (Node *next : map.getNeighbors(current))
            {
                int newCost = costSoFar[current] + 1;
                if (costSoFar.find(next) == costSoFar.end() || newCost < costSoFar[next])
                {
                    costSoFar[next] = newCost;
                    int priority = newCost + heuristic(next, goal);
                    pq.emplace(priority, next);
                    cameFrom[next] = current;
                }
            }
        }
        return reconstructPath(cameFrom, start, goal);
    }

    int heuristic(Node *a, Node *b)
    {
        return abs(a->x - b->x) + abs(a->y - b->y);
    }

    vector<Node *> reconstructPath(unordered_map<Node *, Node *> &cameFrom, Node *start, Node *goal)
    {
        vector<Node *> path;
        for (Node *at = goal; at != nullptr; at = cameFrom[at])
        {
            path.push_back(at);
        }
        reverse(path.begin(), path.end());
        return path;
    }

    void printPath(const vector<Node *> &path)
    {
        cout << "Alternative path:" << endl;
        for (Node *node : path)
        {
            cout << "(" << node->x << ", " << node->y << ") ";
        }
        cout << endl;
    }
};

void resetDistances(Map &map)
{
    for (int i = 0; i < map.getWidth(); ++i)
    {
        for (int j = 0; j < map.getHeight(); ++j)
        {
            map.getNode(i, j)->distanceToGoal = Dmax;
            map.getNode(i, j)->successor = nullptr;
        }
    }
}

void dynamicProgrammingUpdate(Map &map)
{
    queue<Node *> q;
    Node *start = map.getStart();
    Node *goal = map.getGoal();

    if (start)
    {
        start->distanceToGoal = 0;
        q.push(start);
    }

    while (!q.empty())
    {
        Node *current = q.front();
        q.pop();

        for (Node *neighbor : map.getNeighbors(current))
        {
            int newDist = current->distanceToGoal + 1;

            if (newDist < neighbor->distanceToGoal)
            {
                neighbor->distanceToGoal = newDist;
                neighbor->successor = current;
                q.push(neighbor);
            }
        }
    }
}

void dynamicProgrammingUpdateWithTrace(Map &map)
{
    queue<Node *> q;
    Node *start = map.getStart();
    Node *goal = map.getGoal();

    // Initialize the start node's distance to 0 and push it to the queue
    if (start)
    {
        start->distanceToGoal = 0;
        q.push(start);
    }

    while (!q.empty())
    {
        Node *current = q.front();
        q.pop();

        cout << "Current Node: (" << current->x << ", " << current->y << ") Distance: " << current->distanceToGoal << endl;

        for (Node *neighbor : map.getNeighbors(current))
        {
            int newDist = current->distanceToGoal + 1; 

            cout << "Checking Neighbor: (" << neighbor->x << ", " << neighbor->y << ") Current Neighbor Distance: " << neighbor->distanceToGoal << endl;

            if (newDist < neighbor->distanceToGoal)
            {
                neighbor->distanceToGoal = newDist;
                neighbor->successor = current;   
                q.push(neighbor);           

                cout << "Updated Neighbor: (" << neighbor->x << ", " << neighbor->y << ") New Distance: " << newDist << endl;
            }
        }
    }
}

void printShortestPath(Map &map)
{
    Node *current = map.getGoal();
    vector<Node *> path;

    while (current != nullptr)
    {
        path.push_back(current);
        current = current->successor;
    }

    reverse(path.begin(), path.end());

    cout << "Shortest path:" << endl;
    for (Node *node : path)
    {
        cout << "(" << node->x << ", " << node->y << ") ";
    }
    cout << endl;
}

int main()
{
    int width, height;
    cout << "Enter the width & height:";
    cin >> width >> height;
    Map map(width, height);

    int startX, startY, goalX, goalY;
    cout << endl
         << "Enter the starting and end vertices:" << endl;
    cin >> startX >> startY >> goalX >> goalY;
    if (startX < 0 || startX >= width || startY < 0 || startY >= height ||
        goalX < 0 || goalX >= width || goalY < 0 || goalY >= height)
    {
        cout << "Error: Start or goal position is out of bounds." << endl;
        return 1;
    }

    map.setStartAndGoal(startX, startY, goalX, goalY);

    map.setObstacle(3, 4);
    map.setObstacle(3, 5);

    AcceleratingNodes acceleratingNodes(map);
    acceleratingNodes.addAcceleratingNode(2, 2);
    acceleratingNodes.addAcceleratingNode(7, 7);

    resetDistances(map);
    dynamicProgrammingUpdateWithTrace(map);
    printShortestPath(map);
    acceleratingNodes.monitorChanges();

    return 0;
}