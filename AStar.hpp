#pragma once
#include <memory>
#include <vector>
#include <tuple>
#include <opencv2/opencv.hpp>


namespace AStar{
    struct Node {
        int x, y;
        float cost;
        float heuristicCost;
        std::shared_ptr<Node> parent;

        Node(int x, int y, float cost, float heuristicCost,
             std::shared_ptr<Node> parent);
    };


    bool compareNodes(const std::shared_ptr<Node>& a,
                      const std::shared_ptr<Node>& b);

    float euclideanDistance(int x1, int y1, int x2, int y2);

    float diagonalDistance(int x1, int y1, int x2, int y2);

    std::vector<std::tuple<int, int>> findPathAStar(const cv::Mat grid,
                                                    Node start, Node goal);
}