
/*
 * Copyright 2021 The Project Author: lixiang
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "10_hector_rewrite/hector_slam.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/PointCloud2.h"


// 构造函数
HectorMappingRos::HectorMappingRos()
    : private_node_("~"), lastGetMapUpdateIndex(-100), tfB_(0), map_publish_thread_(0)
{
    ROS_INFO_STREAM("\033[1;32m----> Hector SLAM started.\033[0m");

    // 参数初始化
    InitParams();
    // p_scan_subscriber_queue_size_:队列大小,即缓冲的消息数量
    laser_scan_subscriber_ = node_handle_.subscribe(p_scan_topic_, p_scan_subscriber_queue_size_, &HectorMappingRos::scanCallback, this); // 雷达数据处理

    if (p_pub_odometry_)
    {
        std::cout<<"p_pub_odometry_ true"<<std::endl;
        odometryPublisher_ = node_handle_.advertise<nav_msgs::Odometry>("odom_hector", 50);//发布odom_hector话题
    }else{
        std::cout<<"p_pub_odometry_ false"<<std::endl;
    }
    // tf::TransformBroadcaster 是一个发布tf变换的广播器类
    tfB_ = new tf::TransformBroadcaster();

    std::cout<<p_map_size_ <<" "<<p_map_start_x_ <<" "<<p_map_start_y_<<std::endl;
    std::cout<<"p_map_multi_res_levels_:"<<p_map_multi_res_levels_<<std::endl;
    // 世界坐标系(world frame)是一个固定的参考坐标系,通常与机器人固定相对。这里指的是地图左上角位置作为世界坐标系的原点,世界坐标系是原始坐标的，没有分辨率的。
    // p_map_start_x_  p_map_start_y_  就是世界坐标系原点在地图坐标系的位置，地图左上角
    // 地图坐标系(map frame)则是用于构建和表示地图的数据坐标系。这里指的是地图左下角位置作为地图坐标系的原点
    slamProcessor = new hectorslam::HectorSlamProcessor(static_cast<float>(p_map_resolution_), // 地图的分辨率
                                                        p_map_size_, p_map_size_,//地图长宽，单位:栅格个数
                                                        Eigen::Vector2f(p_map_start_x_, p_map_start_y_),//起始坐标 
                                                        p_map_multi_res_levels_);//有几层地图

    /* 如果感知到空间(空白区域),cell状态将更新为 free 的几率。
    这里的几率就是由 p_update_factor_free_ 参数控制的。
    值得注意的是:
    - 参数值越大,更新free状态的可能性就越高
    - 参数值越小,更新free状态的可能性就越小 
    
    p_update_factor_free_取值区间是 0~1 。
    其中:
    - 0 表示cell基本不会更新为free,即保持原状态
    - 1 表示cell几乎一定会更新为free,受感知影响很大
    - 0.4 则表示中等程度的影响
    具体来说:
    - 当超过0.4的置信率时,就认为该cell空置,更新为free
    - 当少于0.4时,则不认为该cell空置,保持原状
    所以p_update_factor_free_=0.4具体表示:
    只有当感知信息的置信度超过40%时,
    cell状态才有可能被更新为free空置状态
    这表明:
    - 设置的阈值不高,容易受感知信息影响
    - 但也不过于放松,有一定的宽容范围
    0.4被看作是一个适中的阈值:
    - 可以使cell状态受感知更新而变化
    - 但又不至于太过轻易地相信感知
    与阈值趋近0或者1相比:
    - 0.4这种中间值会使地图更新得更加适中、灵活而且鲁棒*/
    slamProcessor->setUpdateFactorFree(p_update_factor_free_);                // 0.4

    /* 设定如果感知到空间被占据(非空白区域),cell状态将更新为occupied的几率。
    这里的几率就是由 p_update_factor_occupied_ 参数控制的。
    值得注意的是:
    - 参数值越大,更新occupied状态的可能性就越高
    - 参数值越小,更新occupied状态的可能性就越小 
    p_update_factor_occupied_=0.9 表示只有当感知信息的置信度超过90%时,cell状态才有可能被更新为occupied状态
    */
    slamProcessor->setUpdateFactorOccupied(p_update_factor_occupied_);        // 0.9

    /* 当机器人移动的距离超过 p_map_update_distance_threshold_ 这个阈值时,才会更新地图。 */
    slamProcessor->setMapUpdateMinDistDiff(p_map_update_distance_threshold_); // 0.4 机器人移动的最小距离就会更新地图

    /* 只有当机器人发生的旋转角度超过 p_map_update_angle_threshold_ 这个阈值时,才会更新地图。 */
    slamProcessor->setMapUpdateMinAngleDiff(p_map_update_angle_threshold_);   // 0.9

    // 多层地图的初始化
    int mapLevels = slamProcessor->getMapLevels();//获得地图的层数
    mapLevels = 1; // 这里设置成只发布最高精度的地图，如果有其他需求，如进行路径规划等等需要多层地图时，注释本行。

    std::string mapTopic_ = "map";
    for (int i = 0; i < mapLevels; ++i)
    {
        mapPubContainer.push_back(MapPublisherContainer());
        slamProcessor->addMapMutex(i, new HectorMapMutex());

        std::string mapTopicStr(mapTopic_);

        if (i != 0)
        {
            mapTopicStr.append("_" + boost::lexical_cast<std::string>(i));
        }

        std::string mapMetaTopicStr(mapTopicStr);
        mapMetaTopicStr.append("_metadata");

        MapPublisherContainer &tmp = mapPubContainer[i];
        tmp.mapPublisher_ = node_handle_.advertise<nav_msgs::OccupancyGrid>(mapTopicStr, 1, true);
        tmp.mapMetadataPublisher_ = node_handle_.advertise<nav_msgs::MapMetaData>(mapMetaTopicStr, 1, true);

        setMapInfo(tmp.map_, slamProcessor->getGridMap(i)); // 设置地图服务

        if (i == 0)
        {
            mapPubContainer[i].mapMetadataPublisher_.publish(mapPubContainer[i].map_.map.info);
        }
    }

    // 新建一个线程用来发布地图
    /* HectorMappingRos::publishMapLoop 是线程运行的函数
     * p_map_pub_period_ 是发布地图的频率，单位HZ
     */
    map_publish_thread_ = new boost::thread(boost::bind(&HectorMappingRos::publishMapLoop, this, p_map_pub_period_));
    /* 使 map_to_odom_ 初始化为单位矩阵,即两个坐标系之间的转换被设置为恒等转换。
    setIdentity() 方法将矩阵初始化为单位矩阵,等效于坐标系之间没有偏移和旋转 */
    map_to_odom_.setIdentity();

    // p_base_frame_ -> base_link
    // p_scan_frame_ -> front_laser_link
    // 查找 base_link -> front_laser_link 的tf，循环5次以确保其能找到
    int count = 5;
    ros::Duration dur(0.5);
    while (count-- != 0)
    {
        // 等待base坐标系到scan坐标系的变换,超时时间为0.5秒。
        if (tf_.waitForTransform(p_base_frame_, p_scan_frame_, ros::Time(0), dur))
        {
            // 通过lookupTransform()获取变换结果存入laserTransform_,并跳出循环。
            tf_.lookupTransform(p_base_frame_, p_scan_frame_, ros::Time(0), laserTransform_);
            break;
        }
        else
        {
            ROS_WARN("lookupTransform laser frame into base_link timed out.");
        }
    }

}

HectorMappingRos::~HectorMappingRos()
{
    delete slamProcessor;

    if (tfB_)
        delete tfB_;

    if (map_publish_thread_)
        delete map_publish_thread_;
}

// ros的参数初始化
void HectorMappingRos::InitParams()
{
    private_node_.param("pub_map_baselink_tf", pub_map_to_baselink_tf_, true);
    private_node_.param("pub_map_odom_tf", p_pub_map_odom_transform_, true);
    private_node_.param("pub_odometry_topic", p_pub_odometry_, true);
    private_node_.param("tracking_frame", p_tf_map_scanmatch_transform_frame_name_, std::string("base_link"));

    private_node_.param("scan_topic", p_scan_topic_, std::string("laser_scan"));
    private_node_.param("scan_frame", p_scan_frame_, std::string("front_laser_link"));
    private_node_.param("scan_subscriber_queue_size", p_scan_subscriber_queue_size_, 5);
    private_node_.param("use_max_scan_range", p_use_max_scan_range_, 20.0);

    private_node_.param("map_frame", p_map_frame_, std::string("map"));
    private_node_.param("odom_frame", p_odom_frame_, std::string("odom"));
    private_node_.param("base_frame", p_base_frame_, std::string("base_link"));

    private_node_.param("output_timing", p_timing_output_, false);
    private_node_.param("map_pub_period", p_map_pub_period_, 2.0); // 发布地图的频率，单位HZ

    private_node_.param("map_resolution", p_map_resolution_, 0.05); //地图的分辨率
    private_node_.param("map_size", p_map_size_, 2048);//地图大小，正方形边长：栅格个数
    private_node_.param("map_start_x", p_map_start_x_, 0.5);//起始坐标X，地图边长2048，那么起始坐标0.5就表示起始坐标x是1024
    private_node_.param("map_start_y", p_map_start_y_, 0.5);//起始坐标Y
    private_node_.param("map_multi_res_levels", p_map_multi_res_levels_, 3);//有几层地图

    private_node_.param("update_factor_free", p_update_factor_free_, 0.4);
    private_node_.param("update_factor_occupied", p_update_factor_occupied_, 0.9);

    private_node_.param("map_update_distance_thresh", p_map_update_distance_threshold_, 0.4);
    private_node_.param("map_update_angle_thresh", p_map_update_angle_threshold_, 0.9);

    double tmp = 0.0;
    private_node_.param("laser_min_dist", tmp, 0.2);
    p_sqr_laser_min_dist_ = static_cast<float>(tmp * tmp);// ladar有效据距离最小值的平方

    private_node_.param("laser_max_dist", tmp, 30.0);
    p_sqr_laser_max_dist_ = static_cast<float>(tmp * tmp);// ladar有效据距离最大值的平方

    private_node_.param("laser_z_min_value", tmp, -1.0);
    p_laser_z_min_value_ = static_cast<float>(tmp); // 雷达Z轴数值在base_link下坐标减去雷达Z轴数值最小值

    private_node_.param("laser_z_max_value", tmp, 1.0);
    p_laser_z_max_value_ = static_cast<float>(tmp);
}

// 对ROS地图进行数据初始化与分配内存
void HectorMappingRos::setMapInfo(nav_msgs::GetMap::Response &map_, const hectorslam::GridMap &gridMap)
{
    Eigen::Vector2f mapOrigin(gridMap.getWorldCoords(Eigen::Vector2f::Zero()));
    mapOrigin.array() -= gridMap.getCellLength() * 0.5f;

    map_.map.info.origin.position.x = mapOrigin.x();
    map_.map.info.origin.position.y = mapOrigin.y();
    map_.map.info.origin.orientation.w = 1.0;
    map_.map.info.resolution = gridMap.getCellLength();
    map_.map.info.width = gridMap.getSizeX();
    map_.map.info.height = gridMap.getSizeY();

    map_.map.header.frame_id = p_map_frame_;
    // 分配内存空间
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);
}

/**
 * 激光数据处理回调函数，将ros数据格式转换为算法中的格式，并转换成地图尺度，交由slamProcessor处理。
 * 算法中所有的计算都是在地图尺度下进行。  
 */
void HectorMappingRos::scanCallback(const sensor_msgs::LaserScan &scan)
{
    start_time_ = std::chrono::steady_clock::now();

    ros::WallTime startTime = ros::WallTime::now();

    /*将 scan 转换成 点云格式
    - scan 是输入的激光扫描数据(sensor_msgs/LaserScan) 
    - laser_point_cloud_ 是输出的二维点云(sensor_msgs/PointCloud2)
    - 30.0 是最大有效距离参数,超过该距离的点会被滤除 */
    projector_.projectLaser(scan, laser_point_cloud_, 30.0);

    Eigen::Vector3f startEstimate(Eigen::Vector3f::Zero());

    // 将雷达数据的点云格式 更改成 hector 内部的数据格式
    // slamProcessor->getScaleToMap() 地图的缩放比例，地图分辨率不的倒数，如果分辨率是0.5，也就是一个栅格代表0.5m。倒数就是2，也就是地图方法了2倍
    if (rosPointCloudToDataContainer(laser_point_cloud_, laserTransform_, laserScanContainer, slamProcessor->getScaleToMap()))
    {
        // 首先获取上一帧的位姿，作为初值
        startEstimate = slamProcessor->getLastScanMatchPose();

        // 进入扫描匹配与地图更新
        slamProcessor->update(laserScanContainer, startEstimate);
    }
        
    end_time_ = std::chrono::steady_clock::now();
    time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
    std::cout << "数据转换与扫描匹配用时: " << time_used_.count() << " 秒。" << std::endl;

    if (p_timing_output_)
    {
        ros::WallDuration duration = ros::WallTime::now() - startTime;
        ROS_INFO("HectorSLAM Iter took: %f milliseconds", duration.toSec() * 1000.0f);
    }

    // 更新存储的位姿, 这里的位姿是 base_link 在 map 下的位姿
    poseInfoContainer_.update(slamProcessor->getLastScanMatchPose(), slamProcessor->getLastScanMatchCovariance(), scan.header.stamp, p_map_frame_);

    if (pub_map_to_baselink_tf_)
    {
        // pub map -> odom -> base_link tf
        if (p_pub_map_odom_transform_)
        {
            tfB_->sendTransform(tf::StampedTransform(map_to_odom_, scan.header.stamp, p_map_frame_, p_odom_frame_));
            tfB_->sendTransform(tf::StampedTransform(poseInfoContainer_.getTfTransform(), scan.header.stamp, p_odom_frame_, p_tf_map_scanmatch_transform_frame_name_));
            // tfB_->sendTransform(tf::StampedTransform(map_to_odom_, ros::Time::now(), p_map_frame_, p_odom_frame_));
            // tfB_->sendTransform(tf::StampedTransform(poseInfoContainer_.getTfTransform(), ros::Time::now(), p_odom_frame_, p_tf_map_scanmatch_transform_frame_name_));
        }
        // pub map -> base_link tf
        else
        {
            tfB_->sendTransform(tf::StampedTransform(poseInfoContainer_.getTfTransform(), scan.header.stamp, p_map_frame_, p_tf_map_scanmatch_transform_frame_name_));
        }
    }

    // 发布 odom topic
    if (p_pub_odometry_)
    {
        nav_msgs::Odometry tmp;
        tmp.pose = poseInfoContainer_.getPoseWithCovarianceStamped().pose;
        tmp.header = poseInfoContainer_.getPoseWithCovarianceStamped().header;
        // tmp.header.stamp = ros::Time::now();
        tmp.child_frame_id = p_base_frame_;
        odometryPublisher_.publish(tmp);
    }
    
    end_time_ = std::chrono::steady_clock::now();
    time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
    std::cout << "执行一次回调用时: " << time_used_.count() << " 秒。" << std::endl;
}

// 发布地图的线程
void HectorMappingRos::publishMapLoop(double map_pub_period)
{
    ros::Rate r(1.0 / map_pub_period);//map_pub_period发布地图的频率
    while (ros::ok())
    {
        ros::Time mapTime(ros::Time::now());

        //publishMap(mapPubContainer[2],slamProcessor->getGridMap(2), mapTime);
        //publishMap(mapPubContainer[1],slamProcessor->getGridMap(1), mapTime);
        publishMap(mapPubContainer[0], slamProcessor->getGridMap(0), mapTime, slamProcessor->getMapMutex(0));

        r.sleep();// map_pub_period是2，表示休眠0.5s
    }
}

// 发布ROS地图
void HectorMappingRos::publishMap(MapPublisherContainer &mapPublisher,
                                  const hectorslam::GridMap &gridMap,
                                  ros::Time timestamp, MapLockerInterface *mapMutex)
{
    nav_msgs::GetMap::Response &map_(mapPublisher.map_);

    //only update map if it changed
    if (lastGetMapUpdateIndex != gridMap.getUpdateIndex())
    {
        int sizeX = gridMap.getSizeX();
        int sizeY = gridMap.getSizeY();

        int size = sizeX * sizeY;

        std::vector<int8_t> &data = map_.map.data;

        //std::vector contents are guaranteed to be contiguous, use memset to set all to unknown to save time in loop
        memset(&data[0], -1, sizeof(int8_t) * size);

        if (mapMutex)
        {
            mapMutex->lockMap();
        }

        for (int i = 0; i < size; ++i)
        {
            if (gridMap.isFree(i))
            {
                data[i] = 0;
            }
            else if (gridMap.isOccupied(i))
            {
                data[i] = 100;
            }
        }

        lastGetMapUpdateIndex = gridMap.getUpdateIndex();

        if (mapMutex)
        {
            mapMutex->unlockMap();
        }
    }

    map_.map.header.stamp = timestamp;

    mapPublisher.mapPublisher_.publish(map_.map);
}

/**
 * @brief 将点云数据转换成Hector中雷达数据的格式
 * 
 * @param pointCloud 点云数据
 * @param laserTransform base_link -> front_laser_link tf转换关系
 * @param dataContainer 数据点容器
 * @param scaleToMap 地图的缩放比例
 * @return true 
 * @return false 
 */
bool HectorMappingRos::rosPointCloudToDataContainer(const sensor_msgs::PointCloud &pointCloud, const tf::StampedTransform &laserTransform, hectorslam::DataContainer &dataContainer, float scaleToMap)
{
    size_t size = pointCloud.points.size();
    /*  
    dataContainer.clear();
    getOrigin() 方法可以返回当前坐标变换的原点坐标值
    从 laserTransform 对象中获取原点坐标(x,y,z),并存储到 tf::Vector3 类型的 laserPos 变量中。
    对于一个坐标变换对象来说,其原点坐标表示的是该坐标变换在父坐标系下的位移值
    也就是说,laserTransform 的原点坐标不是在自身坐标系下的(0, 0, 0),而是该坐标系相对于父坐标系的位移量。
    举个例子:
    假设有两个坐标系:
    - base_link 坐标系
    - laser 坐标系
    laser坐标系相对于base_link发生了一个 x=1.0, y=2.0 的位移。
    那么可以创建一个 stamped transform 来表示这个关系:
    tf::StampedTransform laserTransform;
    laserTransform.setOrigin(tf::Vector3(1.0, 2.0, 0.0)); 
    这里设置的原点(1.0, 2.0, 0.0)就是 laser 相对于 base_link 的位移量。
    所以通过 getOrigin() 获得的是 laser 在 base_link 中的坐标位置。
    */
    tf::Vector3 laserPos(laserTransform.getOrigin());

    // dataContainer.setOrigo(Eigen::Vector2f::Zero());
    // 将base_link到雷达坐标系的坐标转换 乘以地图分辨率 当成这帧数据的 origo
    dataContainer.setOrigo(Eigen::Vector2f(laserPos.x(), laserPos.y()) * scaleToMap);

    for (size_t i = 0; i < size; ++i)
    {
        // geometry_msgs::Point32是表示一个3D点的类型
        const geometry_msgs::Point32 &currPoint(pointCloud.points[i]);
        /*
        使用距离平方而不是直接距离,主要有以下两个考虑:
        1. 提高计算效率
        计算平方距离可以避免进行开方运算,从而减少计算量,提高效率。
        例如:
        平方距离: dist_sqr = x*x + y*y
        直接距离: dist = sqrt(x*x + y*y)
        后者需要额外的开方计算。
        2. 避免浮点精度问题
        直接比较两个浮点数的大小,可能存在精度问题,例如:
        float dist1 = 1.23; 
        float dist2 = 1.24;

        if (dist1 < dist2) { // 可能不准确
        }
        但如果比较平方距离,可以避免这个问题:
        float dist_sqr1 = dist1*dist1; 
        float dist_sqr2 = dist2*dist2;

        if (dist_sqr1 < dist_sqr2) { // 正确
        }
        因为误差会被平方放大,从而可比较。
        */
        float dist_sqr = currPoint.x * currPoint.x + currPoint.y * currPoint.y;
        if ((dist_sqr > p_sqr_laser_min_dist_) && (dist_sqr < p_sqr_laser_max_dist_))
        {
            // 0.50开方是0.7071068 
            if ((currPoint.x < 0.0f) && (dist_sqr < 0.50f))
            {
                continue;
            }

            // 距离太远的点跳动太大，如果距离大于使用距离(20m)，就跳过
            if (dist_sqr > p_use_max_scan_range_ * p_use_max_scan_range_)
                continue;
            
            // 点的坐标左乘base_link->laser_link的tf 将得到该点在base_link下的 xy 坐标, 但是z坐标是不正确
            tf::Vector3 pointPosBaseFrame(laserTransform * tf::Vector3(currPoint.x, currPoint.y, currPoint.z));
            
            // 通过再减去 base_link->laser_link的tf的z的值，得到该点在base_link下正确的 z 坐标
            float pointPosLaserFrameZ = pointPosBaseFrame.z() - laserPos.z();

            if (pointPosLaserFrameZ > p_laser_z_min_value_ && pointPosLaserFrameZ < p_laser_z_max_value_)
            {
                // 将雷达数据的 x y 都乘地图的分辨率 0.05 再放入dataContainer中
                dataContainer.add(Eigen::Vector2f(pointPosBaseFrame.x(), pointPosBaseFrame.y()) * scaleToMap);
            }
        }
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hector_slam_rewrite");

    HectorMappingRos hector_slam;

    ros::spin();

    return (0);
}