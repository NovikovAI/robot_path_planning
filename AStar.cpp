#include "AStar.hpp"
#include <queue>
#include <cmath>
#include <algorithm>


AStar::Node::Node(int x = -1, int y = -1, float cost = -1,
                  float heuristicCost = -1,
                  std::shared_ptr<Node> parent = nullptr)
            : x(x), y(y), cost(cost), heuristicCost(heuristicCost),
              parent(parent) {}


bool AStar::compareNodes(const std::shared_ptr<AStar::Node>& a,
                    const std::shared_ptr<AStar::Node>& b) {
    return (a->cost + a->heuristicCost) > (b->cost + b->heuristicCost);
}

float AStar::euclideanDistance(int x1, int y1, int x2, int y2) {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

float AStar::diagonalDistance(int x1, int y1, int x2, int y2) {
    float dx = std::abs(x1 - x2), dy = std::abs(y1 - y2);
    return dx + dy + (std::sqrt(2) - 2) * std::min(dx, dy);
}

std::vector<std::tuple<int, int>> AStar::findPathAStar(const cv::Mat grid,
                                    AStar::Node start, AStar::Node goal) {
    std::vector<std::tuple<int, int>> result;
    if (grid.at<double>(goal.y, goal.x) == 1)
        return result;
    std::vector<AStar::Node> path;
    std::vector<std::vector<bool>> visited(grid.rows,
                                           std::vector<bool>(grid.cols,
                                                             false));
    std::priority_queue<std::shared_ptr<AStar::Node>,
                std::vector<std::shared_ptr<AStar::Node>>,
                decltype(&AStar::compareNodes)> openSet(&AStar::compareNodes);

    openSet.push(std::make_shared<AStar::Node>(start));

    while (!openSet.empty()) {
        std::shared_ptr<AStar::Node> current = openSet.top();
        openSet.pop();

        if (current->x == goal.x && current->y == goal.y) {
            while (current != nullptr) {
                path.push_back(*current);
                current = current->parent;
            }
            break;
        }

        visited[current->y][current->x] = true;

        int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
        int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

        for (int i = 0; i < 8; i++) {
            int newX = current->x + dx[i];
            int newY = current->y + dy[i];

            if ((newX >= 0) && (newX < grid.cols) &&
                (newY >= 0) && (newY < grid.rows) &&
                (grid.at<double>(newY, newX) == 0) && (!visited[newY][newX])) {
                float newCost = current->cost +
                                        AStar::euclideanDistance(current->x,
                                                            current->y,
                                                            newX, newY);
                float heuristicCost = AStar::euclideanDistance(newX, newY,
                                                        goal.x, goal.y);
                std::shared_ptr<AStar::Node> newPtr =
                        std::make_shared<AStar::Node>(newX, newY, newCost,
                                                      heuristicCost, current);
                openSet.push(newPtr);
            }
        }
    }

    std::reverse(path.begin(), path.end());
    for (const AStar::Node& node : path)
        result.push_back(std::tuple{node.x, node.y});
    
    return result;
}