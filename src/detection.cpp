#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>

#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

#include <pcl/registration/ia_ransac.h>

class FeatureCloud
{
public:
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

    FeatureCloud() :
            search_method_xyz_(new SearchMethod),
            normal_radius_(0.02f),
            feature_radius_(0.02f)
    {}

    ~FeatureCloud()
    {}

    // Process the given cloud
    void
    setInputCloud(PointCloud::Ptr xyz)
    {
        xyz_ = xyz;
        processInput();
    }

    // Load and process the cloud in the given PCD file
    void
    loadInputCloud(const std::string &pcd_file)
    {
        xyz_ = PointCloud::Ptr(new PointCloud);
        pcl::io::loadPCDFile(pcd_file, *xyz_);
        processInput();
    }

    // Get a pointer to the cloud 3D points
    PointCloud::Ptr
    getPointCloud() const
    {
        return (xyz_);
    }

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals() const
    {
        return (normals_);
    }

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures() const
    {
        return (features_);
    }

protected:
    // Compute the surface normals and local features
    void
    processInput()
    {
        computeSurfaceNormals();
        computeLocalFeatures();
    }

    // Compute the surface normals
    void
    computeSurfaceNormals()
    {
        normals_ = SurfaceNormals::Ptr(new SurfaceNormals);

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
        norm_est.setInputCloud(xyz_);
        norm_est.setSearchMethod(search_method_xyz_);
        norm_est.setRadiusSearch(normal_radius_);
        norm_est.compute(*normals_);
    }

    // Compute the local feature descriptors
    void
    computeLocalFeatures()
    {
        features_ = LocalFeatures::Ptr(new LocalFeatures);

        pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
        fpfh_est.setInputCloud(xyz_);
        fpfh_est.setInputNormals(normals_);
        fpfh_est.setSearchMethod(search_method_xyz_);
        fpfh_est.setRadiusSearch(feature_radius_);
        fpfh_est.compute(*features_);
    }

private:
    // Point cloud data
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};

class TemplateAlignment
{
public:

    // A struct for storing alignment results
    struct Result
    {
        float fitness_score;
        Eigen::Matrix4f final_transformation;
//        PCL_MAKE_ALIGNED_OPERATOR_NEW
    };

    TemplateAlignment(float min_sample_distance,
                      float max_correspondence_distance,
                      int nr_iterations)
            : min_sample_distance_(min_sample_distance),
              max_correspondence_distance_(max_correspondence_distance),
              nr_iterations_(nr_iterations)
    {
        // Initialize the parameters in the Sample Consensus Initial Alignment (SAC-IA) algorithm
        sac_ia_.setMinSampleDistance(min_sample_distance_);
        sac_ia_.setMaxCorrespondenceDistance(max_correspondence_distance_);
        sac_ia_.setMaximumIterations(nr_iterations_);
    }

    ~TemplateAlignment()
    {}

    // Set the given cloud as the target to which the templates will be aligned
    void
    setTargetCloud(FeatureCloud &target_cloud)
    {
        target_ = target_cloud;
        sac_ia_.setInputTarget(target_cloud.getPointCloud());
        sac_ia_.setTargetFeatures(target_cloud.getLocalFeatures());
    }

    // Add the given cloud to the list of template clouds
    void
    addTemplateCloud(FeatureCloud &template_cloud)
    {
        templates_.push_back(template_cloud);
    }

    // Align the given template cloud to the target specified by setTargetCloud ()
    void
    align(FeatureCloud &template_cloud, TemplateAlignment::Result &result)
    {
        sac_ia_.setInputSource(template_cloud.getPointCloud());
        sac_ia_.setSourceFeatures(template_cloud.getLocalFeatures());

        pcl::PointCloud<pcl::PointXYZ> registration_output;
        sac_ia_.align(registration_output);

        result.fitness_score = (float) sac_ia_.getFitnessScore(max_correspondence_distance_);
        result.final_transformation = sac_ia_.getFinalTransformation();
    }

    // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
    void
    alignAll(std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
    {
        results.resize(templates_.size());
        for (std::size_t i = 0; i < templates_.size(); ++i)
        {
            align(templates_[i], results[i]);
        }
    }

    // Align all of template clouds to the target cloud to find the one with best alignment score
    int
    findBestAlignment(TemplateAlignment::Result &result)
    {
        // Align all of the templates to the target cloud
        std::vector<Result, Eigen::aligned_allocator<Result> > results;
        alignAll(results);

        // Find the template with the best (lowest) fitness score
        float lowest_score = std::numeric_limits<float>::infinity();
        int best_template = 0;
        for (std::size_t i = 0; i < results.size(); ++i)
        {
            const Result &r = results[i];
            if (r.fitness_score < lowest_score)
            {
                lowest_score = r.fitness_score;
                best_template = (int) i;
            }
        }

        // Output the best alignment
        result = results[best_template];
        return (best_template);
    }

private:
    // A list of template clouds and the target to which they will be aligned
    std::vector<FeatureCloud> templates_;
    FeatureCloud target_;

    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    int nr_iterations_;
};

template<typename PointT>
void publishPointCloud(const pcl::PointCloud<PointT> &pcl_cloud,
                       ros::Publisher &publisher,
                       const std::string &frame)
{
    sensor_msgs::PointCloud2::Ptr pc2_cloud(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(pcl_cloud, *pc2_cloud);
    pc2_cloud->header.frame_id = frame;
    pc2_cloud->header.stamp = ros::Time::now();
    publisher.publish(pc2_cloud);
}

template<typename PointT>
void translateToOrigin(const pcl::PointCloud<PointT> &input_cloud, pcl::PointCloud<PointT> &output_cloud)
{
    Eigen::Vector4d origin;
    pcl::compute3DCentroid(input_cloud, origin);
    Eigen::Affine3d transform;
    transform = Eigen::Translation3d{origin.head<3>()};
    pcl::transformPointCloud(input_cloud, output_cloud, transform.inverse());
}

int main(int argc, char *argv[])
{
    /*
     * INITIALIZE ROS NODE
     */
    ros::init(argc, argv, "perception_node");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh_("~");

    /*
     * SET UP PARAMETERS (COULD BE INPUT FROM LAUNCH FILE/TERMINAL)
     */
    auto camera_frame = priv_nh_.param<std::string>("camera_frame", "camera_depth_optical_frame");
    auto world_frame = priv_nh_.param<std::string>("world_frame", "world");
    auto cloud_topic = priv_nh_.param<std::string>("cloud_topic", "kinect/depth/points");
    auto voxel_leaf_size = priv_nh_.param<float>("voxel_leaf_size", 0.002);
    auto plane_max_iter = priv_nh_.param<int>("plane_max_iterations", 50);
    auto plane_dist_thresh = priv_nh_.param<float>("plane_distance_threshold", 0.05);
    auto cluster_tol = priv_nh_.param<float>("cluster_tolerance", 0.01);
    auto cluster_min_size = priv_nh_.param<int>("cluster_min_size", 100);
    auto cluster_max_size = std::numeric_limits<int>::max();
    auto template_model_path = priv_nh_.param<std::string>("template_model", "");
    auto min_sample_distance = priv_nh_.param<float>("min_sample_distance", 0.05f);
    auto max_correspondence_distance = priv_nh_.param<float>("max_corresponding_distance", 0.0001f);
    auto nr_iterations = priv_nh_.param<int>("nr_iterations", 100);

    /*
     * LOAD TEMPLATE CLOUD
     */
    // In order to convert a .stl file to point cloud, first convert it to ply with ctmconv and then run pcl_mesh2pcd.
    pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_template_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(template_model_path, *template_cloud);
    translateToOrigin(*template_cloud, *transformed_template_cloud);
    FeatureCloud template_feature_cloud;
    template_feature_cloud.setInputCloud(transformed_template_cloud);

    /*
     * SETUP PUBLISHERS
     */
    auto plane_pub = nh.advertise<sensor_msgs::PointCloud2>("plane", 1);
    auto not_plane_pub = nh.advertise<sensor_msgs::PointCloud2>("not_plane", 1);
    auto chair_pub = nh.advertise<sensor_msgs::PointCloud2>("chair", 1);
    auto template_pub = nh.advertise<sensor_msgs::PointCloud2>("template", 1);

    constexpr std::size_t num_cluster_pubs = 4;
    std::vector<ros::Publisher> cluster_pubs{num_cluster_pubs};
    for (std::size_t i = 0; i < num_cluster_pubs; i++)
    {
        std::ostringstream topic_name;
        topic_name << "cluster" << i;
        cluster_pubs[i] = nh.advertise<sensor_msgs::PointCloud2>(topic_name.str(), 1);
    }

    while (ros::ok())
    {
        /*
         * LISTEN FOR POINTCLOUD
         */
        std::string topic = nh.resolveName(cloud_topic);
        ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic " << topic);
        sensor_msgs::PointCloud2::ConstPtr recent_cloud =
                ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh);

        /*
         * TRANSFORM POINTCLOUD FROM CAMERA FRAME TO WORLD FRAME
         */
        tf::TransformListener listener;
        tf::StampedTransform stransform;
        try
        {
            listener.waitForTransform(world_frame, recent_cloud->header.frame_id, ros::Time::now(), ros::Duration(6.0));
            listener.lookupTransform(world_frame, recent_cloud->header.frame_id, ros::Time(0), stransform);
        }
        catch (const tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
        }
        sensor_msgs::PointCloud2 transformed_cloud;
        pcl_ros::transformPointCloud(world_frame, stransform, *recent_cloud, transformed_cloud);

        /*
         * CONVERT POINTCLOUD ROS->PCL
         */
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(transformed_cloud, cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud));

        /*
         * VOXEL GRID
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud_ptr);
        voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
        voxel_filter.filter(*cloud_voxel_filtered);
        cloud_ptr = cloud_voxel_filtered;

        ROS_INFO_STREAM("Original cloud had " << cloud_ptr->size() << " points");
        ROS_INFO_STREAM("Downsampled cloud with " << cloud_voxel_filtered->size() << " points");

        /*
         * PLANE SEGEMENTATION
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
        // Create the segmentation object for the planar model and set all the parameters
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(plane_max_iter);
        seg.setDistanceThreshold(plane_dist_thresh);
        // Segment the largest planar component from the cropped cloud
        seg.setInputCloud(cloud_ptr);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            ROS_WARN_STREAM ("Could not estimate a planar model for the given dataset.");
            //break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_ptr);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);
        ROS_INFO_STREAM(
                "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points.");
        publishPointCloud(*cloud_plane, plane_pub, world_frame);

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);
        publishPointCloud(*cloud_f, not_plane_pub, world_frame);

        /*
         * EUCLIDEAN CLUSTER EXTRACTION
         */
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        *cloud_filtered = *cloud_f;
        tree->setInputCloud(cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tol);
        ec.setMinClusterSize(cluster_min_size);
        ec.setMaxClusterSize(cluster_max_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered);
        ec.extract(cluster_indices);

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
        for (std::size_t i = 0; i < std::min(num_cluster_pubs, cluster_indices.size()); i++)
        {
            auto indices = cluster_indices[i];
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (int index : indices.indices)
                cloud_cluster->points.push_back(cloud_filtered->points[index]);
            cloud_cluster->is_dense = true;
            std::cout << "Cluster has " << cloud_cluster->points.size() << " points.\n";
            clusters.push_back(cloud_cluster);
            publishPointCloud(*cloud_cluster, cluster_pubs[i], world_frame);
        }

        auto chair_cloud = clusters.at(0);

        /*
         * STATISTICAL OUTLIER REMOVAL
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(chair_cloud);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        sor.filter(*sor_cloud_filtered);
        chair_cloud = sor_cloud_filtered;

        /*
         * BROADCAST TRANSFORM
         */
        static tf::TransformBroadcaster br;
        tf::Transform part_transform;

        // Here in the tf::Vector3(x,y,z) x,y, and z should be calculated based on the pointcloud filtering results
        part_transform.setOrigin(
                tf::Vector3(chair_cloud->at(1).x, chair_cloud->at(1).y, chair_cloud->at(1).z));
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        part_transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(part_transform, ros::Time::now(), world_frame, "chair"));

        /*
         * CENTER POINT CLOUDS
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_chair_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        translateToOrigin(*chair_cloud, *transformed_chair_cloud);
        publishPointCloud(*transformed_chair_cloud, chair_pub, world_frame);

        /*
         * ALIGN WITH TEMPLATE
         */
        FeatureCloud chair_feature_cloud;
        chair_feature_cloud.setInputCloud(chair_cloud);

        TemplateAlignment align;
        align.addTemplateCloud(template_feature_cloud);
        align.setTargetCloud(chair_feature_cloud);

        TemplateAlignment::Result best_alignment;
        int best_index = align.findBestAlignment(best_alignment);

        Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3, 3>(0, 0);
        Eigen::Vector3f translation = best_alignment.final_transformation.block<3, 1>(0, 3);

        pcl::PointCloud<pcl::PointXYZ> aligned_template_cloud;
        pcl::transformPointCloud(*template_feature_cloud.getPointCloud(),
                                 aligned_template_cloud,
                                 best_alignment.final_transformation);
        publishPointCloud(aligned_template_cloud, template_pub, world_frame);
    }
    return 0;
}