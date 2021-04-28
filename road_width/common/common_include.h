//
// Created by renjie on 2020/11/27.
//

#ifndef ROAD_WIDTH_V2_COMMON_INCLUDE_H
#define ROAD_WIDTH_V2_COMMON_INCLUDE_H
//#define offline_ros
//#define soc

#include<memory>
#include<iostream>
#include<string>
#include<mutex>
#include<vector>
#include<map>
#include<thread>
#include<chrono>
using namespace std;

#include<opencv2/core/core.hpp>   //cv
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
using namespace cv;

#include<eigen3/Eigen/Core>
#include<Eigen/Dense>
using namespace Eigen;

//protobuf
#include <google/protobuf/stubs/common.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>

#ifdef soc
#include"zros/ca_apa_node_transport/ca_apa_agent.pb.h"
#include"zros/live_msgs_oem_changan_s202da/capa_live_msg_mcu_to_soc.h"
#include"zros/core/shared_buffer.h"
#include"zros/common_transport/grid_map.pb.h"
#include"zros/core/transport.h"
//typedef zros::ca_apa_transport::apa_agent::ApaGlobalVehiclePose_T ApaGlobalVehiclePose_T;
typedef zros::common_transport::grid_map::ExtendedOccupancyGrid ExtendedOccupancyGrid;
typedef zros::ca_apa_transport::apa_agent::ApaSlotsInfo_T ApaSlotsInfo_T;
#endif


#ifdef offline_ros
#include<ros/ros.h>
#include<ros/callback_queue.h>
#include<nav_msgs/OccupancyGrid.h>
#include<nav_msgs/Odometry.h>
#include<tf/tf.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>
#include<geometry_msgs/Point.h>
#include<string>
#include<sstream>

template <class M, class T>
ros::SubscribeOptions getSubscribeOptions(
    const std::string &topic, uint32_t queue_size,
    void (T::*fp)(const boost::shared_ptr<M const> &),
    T* obj,
    ros::CallbackQueueInterface* queue,
    const ros::TransportHints &transport_hints = ros::TransportHints()) {
    ros::SubscribeOptions ops;
    ops.template init<M>(topic, queue_size, boost::bind(fp, obj, _1));
    ops.callback_queue = queue;
    ops.transport_hints = transport_hints;
    return ops;
}
#endif




#endif //ROAD_WIDTH_V2_COMMON_INCLUDE_H
