#include <ros/ros.h>
#include <rosbag/bag.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <vector>
#include <algorithm>
#include <string>
#include <iostream>
#include <fstream>

namespace fs = std::filesystem;

using namespace std;

// Function to get sorted PCD file paths
vector<string> getSortedPCDFiles(const string& directory) {
    vector<string> pcd_files;
    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.path().extension() == ".pcd") {
            pcd_files.push_back(entry.path().string());
        }
    }
    // Sort files alphabetically
    sort(pcd_files.begin(), pcd_files.end());
    return pcd_files;
}

// Function to read a PCD file and convert it to a ROS PointCloud2 message
sensor_msgs::PointCloud2 readPCDFile(const string& filepath) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 output;

    cout << "Reading file: " << filepath << endl;

    if (pcl::io::loadPCDFile(filepath, cloud) == -1) {
        cerr << "Error loading file: " << filepath << endl;
        return output;
    }

    // Convert the cloud to ROS message
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "vlp16"; // Set frame ID for RViz visualization
    return output;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "UandBdetect");
    ros::NodeHandle nh;

    string directory = "/mnt/Downloads/cloud_pcd/";
    string bagname = "/mnt/Downloads/mod.bag";

    // Get sorted list of PCD files
    vector<string> pcd_files = getSortedPCDFiles(directory);
    if (pcd_files.empty()) {
        cerr << "No PCD files found in the specified directory: " << directory << endl;
        return -1;
    }

    ofstream fileout(bagname, ios::trunc);
    if (!fileout) {
        cerr << "Failed to create file: " << bagname << endl;
        return -1;
    }
    fileout.close();

    rosbag::Bag bag;
    double time = ros::Time::now().toSec();
    bag.open(bagname, rosbag::bagmode::Write);

    for (size_t i = 0; i < pcd_files.size(); ++i) {
        sensor_msgs::PointCloud2 output = readPCDFile(pcd_files[i]);
        if (output.data.empty()) {
            cerr << "Skipping invalid file: " << pcd_files[i] << endl;
            continue;
        }

        output.header.stamp = ros::Time().fromSec(static_cast<double>(i) / 10.0 + time);
        cout << "Scan time: " << output.header.stamp << endl;
        bag.write("/aicc_LidarFrame", output.header.stamp, output);
    }

    bag.close();
    cout << "Bag file write complete." << endl;

    return 0;
}
