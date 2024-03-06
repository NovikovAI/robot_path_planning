#include <iostream>
#include <array>
#include  "AStar.hpp"   // <cmath> and OpenCV comes from "AStar.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

std::array<float,6> findMinMax(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){
    // NOTE:                     minX,   maxX,  minY,   maxY,  minZ,   maxZ
    std::array<float, 6> ans = {10000, -10000, 10000, -10000, 10000, -10000};
    /*
        Or use bool first = true; to initialize array with first point...
        We'll see.

        "While the at() function is safer than the 
        subscript operator because of its range-checking,
        it’s slightly slower for the same reason."

        I'm sure that I only have 6 elements, so I'll go with array[].
    */
    std::array<float, 3> point;
    for (auto it = cloud->cbegin(); it != cloud->cend(); it++) {
        point = {it->x, it->y, it->z};
        for (int i: {0, 1, 2}){
            if (point[i] < ans[2 * i])
                ans[2 * i] = point[i];
            if (point[i] > ans[2 * i + 1])
                ans[2 * i + 1] = point[i];
        }
    }

    return ans;
}

int main() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
                                        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/user/Desktop/robot_path_planning/CubeTown.pcd",
                         *cloud);

    std::array<float, 6> cloudMinMax = findMinMax(cloud);
    // for (auto item : cloudMinMax)
    //     std::cout << item << " -> " << std::round(item) << std::endl;

    /* 
        Creating an image for occupancy grid.
        Keeping in mind that I will need to store all the float values scaled
        from [minZ, maxZ] to [0, 1] so that this matrix can be cv::imshow'ed.

        x_new = (x - minX) / (maxX - minX)

        Moreover, some of the points in cloud could have the same coordinates.
        Thus, I will be storing height values geq than 'Earth' and
        leq 1.1 meters (the robot's height is 0.6 meters and it needs
        a 0.5 meter gap).
    */
    int height = std::abs(std::round(cloudMinMax[2])) +
                    std::abs(std::round(cloudMinMax[3])) + 1;
    int width = std::abs(std::round(cloudMinMax[0])) +
                    std::abs(std::round(cloudMinMax[1])) + 1;
    cv::Mat occupancyGrid = cv::Mat::zeros(height, width, CV_64F);
    
    std::cout << occupancyGrid.size << std::endl;

    // Using RANSAC to find a plain in point cloud (supposedly 'Earth').
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        std::cerr << "Could not estimate a planar model" <<
                    " for the given dataset." << std::endl;
        return -1;
    }

    std::cout << "Model coefficients: " << coefficients->values[0] << " "
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " "
                                        << coefficients->values[3] << std::endl;

    /*
        Model coefficients show that found plane is parallel to XY
        because it's third coefficient is equal to 1.

        Finding mean Z coordinate for this plane:
    */

    // std::cout << "Model inliers: " << inliers->indices.size() << std::endl;
    double meanZ = 0;
    for (const auto& idx: inliers->indices)
        meanZ += cloud->points[idx].z;
    meanZ /= inliers->indices.size();

    // Filling in the Occupancy Grid:
    for (auto it = cloud->cbegin(); it != cloud->cend(); it++) {
        double tmp;
        if ((it->z > meanZ + 1.1) || (it->z <= meanZ))
            tmp = 0;
        else
            tmp = (it->z - cloudMinMax[4]) / (meanZ + 1.1 - cloudMinMax[4]);
        if (occupancyGrid.at<double>(std::round(it->y) +
                                    std::abs(cloudMinMax[2]),
                                    std::round(it->x) +
                                    std::abs(cloudMinMax[0])) < tmp)
            occupancyGrid.at<double>(std::round(it->y) +
                                    std::abs(cloudMinMax[2]),
                                    std::round(it->x) +
                                    std::abs(cloudMinMax[0])) = tmp;
    }

    cv::Mat OGUpscaled;
    occupancyGrid.copyTo(OGUpscaled);
    for (int i = 0; i < OGUpscaled.rows; i++){
        for (int j = 0; j < OGUpscaled.cols; j++){
            OGUpscaled.at<double>(i, j) =
                                (OGUpscaled.at<double>(i, j) <= 0.15) ? 0 : 1;
        }
    }

    // my A* implementation for path finding
    AStar::Node goal(48, 46, 0, 0, nullptr);
    int startX = 40, startY = 35;
    AStar::Node start(startX, startY, 0, 
                      AStar::euclideanDistance(startX, startY, goal.x, goal.y),
                      nullptr);
    std::vector<std::tuple<int, int>> path = AStar::findPathAStar(OGUpscaled,
                                                                  start, goal);
    if (!path.empty()){
        std::cout << "Found path: (X, Y)" << std::endl;
        for (auto node : path) {
            std::cout << "(" << std::get<0>(node) <<
                        ", " << std::get<1>(node) << ") ";
        }
        std::cout << std::endl;
    } else {
        std::cout << "Unable to find a path." << std::endl;
    }

    // making a color image
    OGUpscaled.convertTo(OGUpscaled, CV_8UC1, 255);
    cv::cvtColor(OGUpscaled, OGUpscaled, cv::COLOR_GRAY2BGR);

    cv::Mat tmpMat;
    cv::resize(OGUpscaled, tmpMat,
                cv::Size(width * 5, height * 5), cv::INTER_LINEAR);
    cv::imwrite("/home/user/Desktop/robot_path_planning/img/grid.jpg",
                OGUpscaled);
    cv::imwrite("/home/user/Desktop/robot_path_planning/img/grid_upscaled.jpg",
                tmpMat);

    // drawing found path
    for (auto idx : path) {
        OGUpscaled.at<cv::Vec3b>(std::get<1>(idx),
                                 std::get<0>(idx)) = cv::Vec3b(0, 0, 255);
    }
    
    cv::resize(OGUpscaled, tmpMat,
                cv::Size(width * 5, height * 5), cv::INTER_LINEAR);
    cv::imshow("Occupancy Grid", tmpMat);
    cv::waitKey();
    cv::imwrite("/home/user/Desktop/robot_path_planning/img/path.jpg",
                OGUpscaled);
    cv::imwrite("/home/user/Desktop/robot_path_planning/img/path_upscaled.jpg",
                tmpMat);
    cv::destroyAllWindows();

    /*      WORKING NOTES:
        The first image that I got shows that maxZ
        might be from an outlier of point cloud.
        
        Solution: instead of scaling from [minZ, maxZ]
        I am scaling from [minZ, meanZ + 1.1].

        I am making OGUpscaled into an actual
        occupancy grid by making it binary.
        While doing so I also filter some noise
        which is all values leq than 0.15
        (which is 16.5 santimeters... might be too high but that is what gives
         me a decent grid/image from this particular point cloud).
    */
    
    return 0;
}
