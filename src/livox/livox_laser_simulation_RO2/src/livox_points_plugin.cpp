#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/logging.hpp>

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/MultiRayShape.hh>// Store the latest laser scans into laserMsg
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/Node.hh>
#include <chrono>
#include "ros2_livox/livox_points_plugin.h"
#include "ros2_livox/csv_reader.hpp"
#include "ros2_livox/livox_ode_multiray_shape.h"
#include <livox_ros_driver2/msg/custom_msg.hpp>

namespace gazebo
{

    GZ_REGISTER_SENSOR_PLUGIN(LivoxPointsPlugin)

    LivoxPointsPlugin::LivoxPointsPlugin() {}

    LivoxPointsPlugin::~LivoxPointsPlugin() {}

    void convertDataToRotateInfo(const std::vector<std::vector<double>> &datas, std::vector<AviaRotateInfo> &avia_infos)
    {
        avia_infos.reserve(datas.size());
        double deg_2_rad = M_PI / 180.0;
        for (auto &data : datas)
        {
            if (data.size() == 3)
            {
                avia_infos.emplace_back();
                avia_infos.back().time = data[0];
                avia_infos.back().azimuth = data[1] * deg_2_rad;
                avia_infos.back().zenith = data[2] * deg_2_rad - M_PI_2; //转化成标准的右手系角度
            } else {
            RCLCPP_ERROR(rclcpp::get_logger("convertDataToRotateInfo"), "data size is not 3!");
        }
        }
    }

    void LivoxPointsPlugin::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr sdf)
    {
        node_ = gazebo_ros::Node::Get(sdf);
        
        std::vector<std::vector<double>> datas;
        std::string file_name = sdf->Get<std::string>("csv_file_name");
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "load csv file name: %s", file_name.c_str());
        if (!CsvReader::ReadCsvFile(file_name, datas))
        {   
            RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "cannot get csv file! %s will return !", file_name.c_str());
            return;
        }
        sdfPtr = sdf;
        auto rayElem = sdfPtr->GetElement("ray");
        auto scanElem = rayElem->GetElement("scan");
        auto rangeElem = rayElem->GetElement("range");


        raySensor = _parent;
        auto sensor_pose = raySensor->Pose();
        auto curr_scan_topic = sdf->Get<std::string>("topic");
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "ros topic name: %s", curr_scan_topic.c_str());

        child_name = raySensor->Name();
        parent_name = raySensor->ParentName();
        size_t delimiter_pos = parent_name.find("::");
        parent_name = parent_name.substr(delimiter_pos + 2);

        node = transport::NodePtr(new transport::Node());
        node->Init(raySensor->WorldName());
        // PointCloud2 publisher
        cloud2_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(curr_scan_topic + "/lidar", 10);
        // CustomMsg publisher
        custom_pub = node_->create_publisher<livox_ros_driver2::msg::CustomMsg>(curr_scan_topic, 10);

        scanPub = node->Advertise<msgs::LaserScanStamped>(curr_scan_topic+"laserscan", 50);

        aviaInfos.clear();
        convertDataToRotateInfo(datas, aviaInfos);
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "scan info size: %ld", aviaInfos.size());
        maxPointSize = aviaInfos.size();

        RayPlugin::Load(_parent, sdfPtr);
        laserMsg.mutable_scan()->set_frame(_parent->ParentName());
        // parentEntity = world->GetEntity(_parent->ParentName());
        parentEntity = this->world->EntityByName(_parent->ParentName());
        //SendRosTf(sensor_pose, raySensor->ParentName(), raySensor->Name());
        auto physics = world->Physics();
        laserCollision = physics->CreateCollision("multiray", _parent->ParentName());
        laserCollision->SetName("ray_sensor_collision");
        laserCollision->SetRelativePose(_parent->Pose());
        laserCollision->SetInitialRelativePose(_parent->Pose());
        rayShape.reset(new gazebo::physics::LivoxOdeMultiRayShape(laserCollision));
        laserCollision->SetShape(rayShape);
        samplesStep = sdfPtr->Get<int>("samples");
        downSample = sdfPtr->Get<int>("downsample");
        if (downSample < 1)
        {
            downSample = 1;
        }
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "sample: %ld", samplesStep);
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "downsample: %ld", downSample);
        rayShape->RayShapes().reserve(samplesStep / downSample);
        rayShape->Load(sdfPtr);
        rayShape->Init();
        minDist = rangeElem->Get<double>("min");
        maxDist = rangeElem->Get<double>("max");
        auto offset = laserCollision->RelativePose();
        ignition::math::Vector3d start_point, end_point;
        for (int j = 0; j < samplesStep; j += downSample)
        {
            int index = j % maxPointSize;
            auto &rotate_info = aviaInfos[index];
            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            start_point = minDist * axis + offset.Pos();
            end_point = maxDist * axis + offset.Pos();
            rayShape->AddRay(start_point, end_point);
        }
    }
void LivoxPointsPlugin::OnNewLaserScans()
{
    if (rayShape)
    {
        std::vector<std::pair<int, AviaRotateInfo>> points_pair;
        InitializeRays(points_pair, rayShape);
        rayShape->Update();

        // 初始化激光扫描消息（不变）
        msgs::Set(laserMsg.mutable_time(), world->SimTime());
        msgs::LaserScan *scan = laserMsg.mutable_scan();
        InitializeScan(scan);

        // 初始化 Livox CustomMsg（不变）
        livox_ros_driver2::msg::CustomMsg pp_livox;
        pp_livox.header.stamp = node_->get_clock()->now();
        pp_livox.header.frame_id = raySensor->Name();
        int count = 0;
        boost::chrono::high_resolution_clock::time_point start_time = boost::chrono::high_resolution_clock::now();

        // 初始化 PointCloud2（新代码）
        sensor_msgs::msg::PointCloud2 cloud2;
        cloud2.header.stamp = pp_livox.header.stamp;
        cloud2.header.frame_id = pp_livox.header.frame_id;
        cloud2.height = 1;
        cloud2.width = 0;
        cloud2.is_dense = false;

        // 定义字段（x, y, z, intensity）
        sensor_msgs::msg::PointField field_x, field_y, field_z, field_intensity;
        field_x.name = "x";
        field_x.offset = 0;
        field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_x.count = 1;

        field_y.name = "y";
        field_y.offset = 4;
        field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_y.count = 1;

        field_z.name = "z";
        field_z.offset = 8;
        field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_z.count = 1;

        field_intensity.name = "intensity";
        field_intensity.offset = 12;
        field_intensity.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_intensity.count = 1;

        cloud2.fields = {field_x, field_y, field_z, field_intensity};
        cloud2.point_step = 16;
        std::vector<uint8_t> cloud_data;
        cloud_data.reserve(points_pair.size() * cloud2.point_step);

        // 循环处理每个点
        for (auto &pair : points_pair)
        {
            auto range = rayShape->GetRange(pair.first);
            auto intensity = rayShape->GetRetro(pair.first);

            if (range >= RangeMax() || range <= RangeMin())
                range = 0;

            // 计算点坐标
            auto rotate_info = pair.second;
            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            auto point = range * axis;

            // 填充 CustomMsg
            livox_ros_driver2::msg::CustomPoint p;  // 确保 p 在此处声明
            p.x = point.X();
            p.y = point.Y();
            p.z = point.Z();
            p.reflectivity = intensity;

            // 计算时间偏移
            boost::chrono::high_resolution_clock::time_point end_time = boost::chrono::high_resolution_clock::now();
            boost::chrono::nanoseconds elapsed_time = boost::chrono::duration_cast<boost::chrono::nanoseconds>(end_time - start_time);
            p.offset_time = elapsed_time.count();  // 现在 p 已声明，可正常使用
            pp_livox.points.push_back(p);
            count++;

            // 填充 PointCloud2 数据
            float x = point.X();
            float y = point.Y();
            float z = point.Z();
            float inten = intensity;

            auto write_float = [&](float val) {
                const uint8_t *ptr = reinterpret_cast<const uint8_t*>(&val);
                cloud_data.insert(cloud_data.end(), ptr, ptr + 4);
            };
            write_float(x);
            write_float(y);
            write_float(z);
            write_float(inten);
            cloud2.width++;
        }

        // 发布消息
        if (scanPub && scanPub->HasConnections()) 
            scanPub->Publish(laserMsg);
        pp_livox.point_num = count;
        custom_pub->publish(pp_livox);

        cloud2.row_step = cloud2.point_step * cloud2.width;
        cloud2.data = cloud_data;
        cloud2_pub->publish(cloud2);
    }
}

//     void LivoxPointsPlugin::OnNewLaserScans()
// {
//     // Check if rayShape has been initialized
//     if (rayShape)
//     {
//         std::vector<std::pair<int, AviaRotateInfo>> points_pair;
//         // Initialize ray scan point pairs
//         InitializeRays(points_pair, rayShape);
//         rayShape->Update();

//         // Create laser scan message and set the timestamp
//         msgs::Set(laserMsg.mutable_time(), world->SimTime());
//         msgs::LaserScan *scan = laserMsg.mutable_scan();
//         InitializeScan(scan);

//         // Create a custom message pp_livox for publishing Livox CustomMsg type messages
//         livox_ros_driver2::msg::CustomMsg pp_livox;
//         pp_livox.header.stamp = node_->get_clock()->now();
//         pp_livox.header.frame_id = raySensor->Name();
//         int count = 0;
//         boost::chrono::high_resolution_clock::time_point start_time = boost::chrono::high_resolution_clock::now();

//         // For publishing PointCloud2 type messages
//         sensor_msgs::msg::PointCloud cloud;
//         cloud.header.stamp = node_->get_clock()->now();
//         cloud.header.frame_id = raySensor->Name();
//         auto &clouds = cloud.points;

//         // Iterate over ray scan point pairs
//         for (auto &pair : points_pair)
//         {
//             auto range = rayShape->GetRange(pair.first);
//             auto intensity = rayShape->GetRetro(pair.first);

//             // Handle out-of-range data
//             if (range >= RangeMax())
//             {
//                 range = 0;
//             }
//             else if (range <= RangeMin())
//             {
//                 range = 0;
//             }

//             // Calculate point cloud data
//             auto rotate_info = pair.second;
//             ignition::math::Quaterniond ray;
//             ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
//             auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
//             auto point = range * axis;

//             // Fill the CustomMsg point cloud message
//             livox_ros_driver2::msg::CustomPoint p;
//             p.x = point.X();
//             p.y = point.Y();
//             p.z = point.Z();
//             p.reflectivity = intensity;

//             // Fill the PointCloud point cloud message
//             clouds.emplace_back();
//             clouds.back().x = point.X();
//             clouds.back().y = point.Y();
//             clouds.back().z = point.Z();

//             // Fill the PointCloud point cloud message
//             clouds.emplace_back();
//             clouds.back().x = point.X();
//             clouds.back().y = point.Y();
//             clouds.back().z = point.Z();

//             // Calculate timestamp offset
//             boost::chrono::high_resolution_clock::time_point end_time = boost::chrono::high_resolution_clock::now();
//             boost::chrono::nanoseconds elapsed_time = boost::chrono::duration_cast<boost::chrono::nanoseconds>(end_time - start_time);
//             p.offset_time = elapsed_time.count();

//             // Add point cloud data to the CustomMsg message
//             pp_livox.points.push_back(p);
//             count++;
//         }

//         if (scanPub && scanPub->HasConnections()) scanPub->Publish(laserMsg);

//         // Set the number of point cloud data and publish the CustomMsg message
//         pp_livox.point_num = count;
//         custom_pub->publish(pp_livox);

//         // Publish PointCloud2 type message
//         sensor_msgs::msg::PointCloud2 cloud2;
//         sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);
//         cloud2.header = cloud.header;
//         cloud2_pub->publish(cloud2);
//     }
// }

    void LivoxPointsPlugin::InitializeRays(std::vector<std::pair<int, AviaRotateInfo>> &points_pair,
                                           boost::shared_ptr<physics::LivoxOdeMultiRayShape> &ray_shape)
    {
        auto &rays = ray_shape->RayShapes();
        ignition::math::Vector3d start_point, end_point;
        ignition::math::Quaterniond ray;
        auto offset = laserCollision->RelativePose();
        int64_t end_index = currStartIndex + samplesStep;
        long unsigned int ray_index = 0;
        auto ray_size = rays.size();
        points_pair.reserve(rays.size());
        for (int k = currStartIndex; k < end_index; k += downSample)
        {
            auto index = k % maxPointSize;
            auto &rotate_info = aviaInfos[index];
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            start_point = minDist * axis + offset.Pos();
            end_point = maxDist * axis + offset.Pos();
            if (ray_index < ray_size)
            {
                rays[ray_index]->SetPoints(start_point, end_point);
                points_pair.emplace_back(ray_index, rotate_info);
            }
            ray_index++;
        }
        currStartIndex += samplesStep;
    }

    void LivoxPointsPlugin::InitializeScan(msgs::LaserScan *&scan)
    {
        // Store the latest laser scans into laserMsg
        msgs::Set(scan->mutable_world_pose(), raySensor->Pose() + parentEntity->WorldPose());
        scan->set_angle_min(AngleMin().Radian());
        scan->set_angle_max(AngleMax().Radian());
        scan->set_angle_step(AngleResolution());
        scan->set_count(RangeCount());

        scan->set_vertical_angle_min(VerticalAngleMin().Radian());
        scan->set_vertical_angle_max(VerticalAngleMax().Radian());
        scan->set_vertical_angle_step(VerticalAngleResolution());
        scan->set_vertical_count(VerticalRangeCount());

        scan->set_range_min(RangeMin());
        scan->set_range_max(RangeMax());

        scan->clear_ranges();
        scan->clear_intensities();

        unsigned int rangeCount = RangeCount();
        unsigned int verticalRangeCount = VerticalRangeCount();

        for (unsigned int j = 0; j < verticalRangeCount; ++j)
        {
            for (unsigned int i = 0; i < rangeCount; ++i)
            {
                scan->add_ranges(0);
                scan->add_intensities(0);
            }
        }
    }

    ignition::math::Angle LivoxPointsPlugin::AngleMin() const
    {
        if (rayShape)
            return rayShape->MinAngle();
        else
            return -1;
    }

    ignition::math::Angle LivoxPointsPlugin::AngleMax() const
    {
        if (rayShape)
        {
            return ignition::math::Angle(rayShape->MaxAngle().Radian());
        }
        else
            return -1;
    }

    double LivoxPointsPlugin::GetRangeMin() const { return RangeMin(); }

    double LivoxPointsPlugin::RangeMin() const
    {
        if (rayShape)
            return rayShape->GetMinRange();
        else
            return -1;
    }

    double LivoxPointsPlugin::GetRangeMax() const { return RangeMax(); }

    double LivoxPointsPlugin::RangeMax() const
    {
        if (rayShape)
            return rayShape->GetMaxRange();
        else
            return -1;
    }

    double LivoxPointsPlugin::GetAngleResolution() const { return AngleResolution(); }

    double LivoxPointsPlugin::AngleResolution() const { return (AngleMax() - AngleMin()).Radian() / (RangeCount() - 1); }

    double LivoxPointsPlugin::GetRangeResolution() const { return RangeResolution(); }

    double LivoxPointsPlugin::RangeResolution() const
    {
        if (rayShape)
            return rayShape->GetResRange();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetRayCount() const { return RayCount(); }

    int LivoxPointsPlugin::RayCount() const
    {
        if (rayShape)
            return rayShape->GetSampleCount();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetRangeCount() const { return RangeCount(); }

    int LivoxPointsPlugin::RangeCount() const
    {
        if (rayShape)
            return rayShape->GetSampleCount() * rayShape->GetScanResolution();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetVerticalRayCount() const { return VerticalRayCount(); }

    int LivoxPointsPlugin::VerticalRayCount() const
    {
        if (rayShape)
            return rayShape->GetVerticalSampleCount();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetVerticalRangeCount() const { return VerticalRangeCount(); }

    int LivoxPointsPlugin::VerticalRangeCount() const
    {
        if (rayShape)
            return rayShape->GetVerticalSampleCount() * rayShape->GetVerticalScanResolution();
        else
            return -1;
    }

    ignition::math::Angle LivoxPointsPlugin::VerticalAngleMin() const
    {
        if (rayShape)
        {
            return ignition::math::Angle(rayShape->VerticalMinAngle().Radian());
        }
        else
            return -1;
    }

    ignition::math::Angle LivoxPointsPlugin::VerticalAngleMax() const
    {
        if (rayShape)
        {
            return ignition::math::Angle(rayShape->VerticalMaxAngle().Radian());
        }
        else
            return -1;
    }

    double LivoxPointsPlugin::GetVerticalAngleResolution() const { return VerticalAngleResolution(); }

    double LivoxPointsPlugin::VerticalAngleResolution() const
    {
        return (VerticalAngleMax() - VerticalAngleMin()).Radian() / (VerticalRangeCount() - 1);
    }


}
