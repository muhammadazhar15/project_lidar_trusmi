#include <rclcpp/rclcpp.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/kdtree.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

#include <thread>

class PoissonReconstructionNode : public rclcpp::Node
{
public:
    PoissonReconstructionNode()
        : Node("scanner_volume")
    {
        std::string package_name = "scanner_volume";
        // Get installed package share directory
        std::string share_dir = ament_index_cpp::get_package_share_directory(package_name);
        // Append your folder
        std::filesystem::path pcd_path = std::filesystem::path(share_dir) / "pcd_data" / "sphere_1r.pcd";
        
        this->declare_parameter<std::string>("pcd_file", pcd_path.string());
        std::string filename = this->get_parameter("pcd_file").as_string();

        processPointCloud(filename);
    }

private:
    void processPointCloud(const std::string &filename)
    {
        // Load point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file %s", filename.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %lu points", cloud->size());

        // Estimate normals
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        ne.setInputCloud(cloud);
        ne.setSearchMethod(tree);
        // ne.setKSearch(20);
        ne.setRadiusSearch(0.1);   // 10 cm
        ne.compute(*normals);

        // Combine XYZ + Normals
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

        // Poisson reconstruction
        pcl::Poisson<pcl::PointNormal> poisson;
        poisson.setDepth(9);              // Increase for higher resolution
        poisson.setSolverDivide(8);
        poisson.setIsoDivide(8);
        poisson.setPointWeight(2.0f);
        poisson.setInputCloud(cloud_with_normals);

        pcl::PolygonMesh mesh;
        poisson.reconstruct(mesh);

        if (mesh.polygons.empty()){
            RCLCPP_WARN(this->get_logger(), "Mesh has no polygons!");
        }
        else{
            double volume = computeMeshVolume(mesh);
            RCLCPP_INFO(this->get_logger(), "Mesh Volume: %.6f cubic meters", volume);

            RCLCPP_INFO(this->get_logger(), "Poisson reconstruction completed.");
            visualizeMesh(mesh);
        }
    }

    void visualizeMesh(pcl::PolygonMesh &mesh)
    {
        pcl::visualization::PCLVisualizer::Ptr viewer(
            new pcl::visualization::PCLVisualizer("Poisson Reconstruction"));

        viewer->setBackgroundColor(0, 0, 0);
        viewer->addPolygonMesh(mesh, "mesh");
        viewer->setRepresentationToSurfaceForAllActors();

        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    double computeMeshVolume(const pcl::PolygonMesh& mesh)
    {
        // Convert mesh cloud to PointXYZ
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromPCLPointCloud2(mesh.cloud, cloud);

        double volume = 0.0;

        for (const auto& polygon : mesh.polygons)
        {
            if (polygon.vertices.size() != 3)
                continue; // Skip non-triangle faces

            const Eigen::Vector3d v0(
                cloud.points[polygon.vertices[0]].x,
                cloud.points[polygon.vertices[0]].y,
                cloud.points[polygon.vertices[0]].z);

            const Eigen::Vector3d v1(
                cloud.points[polygon.vertices[1]].x,
                cloud.points[polygon.vertices[1]].y,
                cloud.points[polygon.vertices[1]].z);

            const Eigen::Vector3d v2(
                cloud.points[polygon.vertices[2]].x,
                cloud.points[polygon.vertices[2]].y,
                cloud.points[polygon.vertices[2]].z);

            volume += v0.dot(v1.cross(v2));
        }

        return std::abs(volume) / 6.0;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoissonReconstructionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}