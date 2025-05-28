#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include "rover_msgs/msg/exteroception.hpp"

#include <cmath>  // for std::isnan
#include <Eigen/Dense>

class PointCloudConverter : public rclcpp::Node
{
  public:
  PointCloudConverter() : Node("pointcloud_converter_node")
  {
    sub_pointcloud2 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/zed/zed_node/point_cloud/cloud_registered", 10,
      std::bind(&PointCloudConverter::callback, this, std::placeholders::_1));

    pub_sparse = this->create_publisher<sensor_msgs::msg::PointCloud>("/sparse_pointcloud", 10);
    pub_dense = this->create_publisher<sensor_msgs::msg::PointCloud>("/dense_pointcloud", 10);
    pub_exteroception = this->create_publisher<rover_msgs::msg::Exteroception>("/exteroception", 10);
    last_time_ = this->now();  // 초기화
  }

  private:
  rclcpp::Time last_time_;
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto first_time = this->now();
    double time_diff_sec = (first_time-last_time_).seconds();
    last_time_ = first_time;

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);

    // 1. 좌표변환 행렬 정의
    // {r} : robot_basefootprint 좌표계, {c} : camera 좌표계
    // T_rc : {r}에서 {c}로의 좌표계 변환

    Eigen::Matrix4f transform;
    float alpha = 0.0*M_PI/180;
    float beta = 15.0*M_PI/180;
    float gamma = 0.0*M_PI/180;
    float tx = 0.15, ty = 0.0, tz = 0.45;
    transform << cos(alpha)*cos(beta), cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma), cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma), tx,
                 sin(alpha)*cos(beta), sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma), sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma), ty,
                 -sin(beta), cos(beta)*sin(gamma), cos(beta)*cos(gamma), tz,
                 0, 0, 0,  1;

    // 2. 좌표계 변환 실행
    pcl::PointCloud<pcl::PointXYZ> pcl_transformed;
    pcl::transformPointCloud(pcl_cloud, pcl_transformed, transform);
    
    // 3. Voxel Grid 필터 적용
    pcl::PointCloud<pcl::PointXYZ> sparse_pcl_voxelized;
    pcl::PointCloud<pcl::PointXYZ> dense_pcl_voxelized;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;

    voxel_filter.setInputCloud(pcl_transformed.makeShared());
    voxel_filter.setLeafSize(0.1, 0.1, 0.1);  // 10cm voxel
    voxel_filter.filter(sparse_pcl_voxelized);

    voxel_filter.setInputCloud(pcl_transformed.makeShared());
    voxel_filter.setLeafSize(0.04, 0.04, 0.04);  // 4cm voxel
    voxel_filter.filter(dense_pcl_voxelized);

    // 4. Heightmap 만들기
    float center_x = 1.0, center_y = 0.0;
    float res_sparse = 0.1, res_dense = 0.04;
    float len_sparse = 2.0, len_dense = 1.0;
    float sparse_max_x = center_x + len_sparse/2 + res_sparse;
    float sparse_max_y = center_y + len_sparse/2 + res_sparse;
    float sparse_min_x = center_x - len_sparse/2 - res_sparse;
    float sparse_min_y = center_y - len_sparse/2 - res_sparse;
    float dense_max_x = center_x + len_dense/2 + res_dense;
    float dense_max_y = center_y + len_dense/2 + res_dense;
    float dense_min_x = center_x - len_dense/2 - res_dense;
    float dense_min_y = center_y - len_dense/2 - res_dense;

    // sparse_voxelize된 point를 임시로 섹터별로 구분하기 위해 필요한 변수들 및 작업
    int sparse_x_index, sparse_y_index, sparse_ref_index, sparse_voxel_index;
    int sparse_store_len = pow(int(len_sparse/res_sparse)+2, 2);
    std::vector<float> sparse_voxel_height_store(sparse_store_len, 0.0);

    // dense_voxelize된 point를 임시로 섹터별로 구분하기 위해 필요한 변수들 및 작업
    int dense_x_index, dense_y_index, dense_ref_index, dense_voxel_index;
    int dense_store_len = pow(int(len_dense/res_dense)+2, 2);
    std::vector<float> dense_voxel_height_store(dense_store_len, 0.0);

    // publish할 sparse_point
    sparse_store_len = pow(int(len_sparse/res_sparse)+1, 2);
    std::vector<float> sparse_heightmap(sparse_store_len, 0);

    // publish할 dense_point
    dense_store_len = pow(int(len_dense/res_dense)+1, 2);
    std::vector<float> dense_heightmap(dense_store_len, 0);

    // 변환된 결과를 sensor_msgs::PointCloud로 변환
    // 변환된 결과를 msg::dense로 변환
    // 변환된 결과를 msg::sparse로 변환
    sensor_msgs::msg::PointCloud dense_cloud;
    dense_cloud.header = msg->header;
    dense_cloud.header.frame_id = "map";

    sensor_msgs::msg::PointCloud sparse_cloud;
    sparse_cloud.header = msg->header;
    sparse_cloud.header.frame_id = "map";

    rover_msgs::msg::Exteroception exteroception_msg;

    // sparse_voxel point를 섹터별로 분류
    for (const auto& pt : sparse_pcl_voxelized.points) {
      if (!std::isfinite(pt.x) || pt.x < sparse_min_x || pt.x > sparse_max_x || pt.y < sparse_min_y || pt.y > sparse_max_y) {
        continue;  // NaN이나 inf, 경계를 넘는 point들은 다 제외
      }
      sparse_x_index = static_cast<int>((pt.x-sparse_min_x)/res_sparse);
      sparse_y_index = static_cast<int>((pt.y-sparse_min_y)/res_sparse);
      sparse_ref_index = (int(len_sparse/res_sparse)+2)*sparse_y_index + sparse_x_index;
      
      // 섹터별로 voxel point cloud 저장하기
      // 섹터별로 가장 높은 height만 저장
      if(sparse_voxel_height_store[sparse_ref_index] < pt.z){
        sparse_voxel_height_store[sparse_ref_index] = pt.z;
      }
    }

    // dense_voxel point를 섹터별로 분류
    for (const auto& pt : dense_pcl_voxelized.points) {
      if (!std::isfinite(pt.x) || pt.x < dense_min_x || pt.x > dense_max_x || pt.y < dense_min_y || pt.y > dense_max_y) {
        continue;  // NaN이나 inf, 경계를 넘는 point들은 다 제외
      }
      dense_x_index = static_cast<int>((pt.x-dense_min_x)/res_dense);
      dense_y_index = static_cast<int>((pt.y-dense_min_y)/res_dense);
      dense_ref_index = (int(len_dense/res_dense)+2)*dense_y_index + dense_x_index;
      
      // 섹터별로 voxel point cloud 저장하기
      // 섹터별로 가장 높은 height만 저장
      if(dense_voxel_height_store[dense_ref_index] < pt.z){
        dense_voxel_height_store[dense_ref_index] = pt.z;
      }
    }

    // sparse heightmap에 담기
    float x, y;
    float start_x = center_x-len_sparse/2;
    float start_y = center_y-len_sparse/2;
    geometry_msgs::msg::Point32 p;

    for (int index = 0; index<sparse_store_len; index++) {
      sparse_voxel_index = index + index/(int(len_sparse/res_sparse)+1);
      sparse_heightmap[index] = (sparse_voxel_height_store[sparse_voxel_index] + sparse_voxel_height_store[sparse_voxel_index+1] + sparse_voxel_height_store[sparse_voxel_index+(int(len_sparse/res_sparse)+2)] + sparse_voxel_height_store[sparse_voxel_index+(int(len_sparse/res_sparse)+3)])/4;
      x = start_x + (index%(int(len_sparse/res_sparse)+1))*res_sparse;
      y = start_y + (index/(int(len_sparse/res_sparse)+1))*res_sparse;
      p.x = x;
      p.y = y;
      p.z = sparse_heightmap[index];
      sparse_cloud.points.push_back(p);
      exteroception_msg.sparse.push_back(p.z);
    }

    // dense heightmap에 담기
    start_x = center_x-len_dense/2;
    start_y = center_y-len_dense/2;

    for (int index = 0; index<dense_store_len; index++) {
      dense_voxel_index = index + index/(int(len_dense/res_dense)+1);
      dense_heightmap[index] = (dense_voxel_height_store[dense_voxel_index] + dense_voxel_height_store[dense_voxel_index+1] + dense_voxel_height_store[dense_voxel_index+(int(len_dense/res_dense)+2)] + dense_voxel_height_store[dense_voxel_index+(int(len_dense/res_dense)+3)])/4;
      x = start_x + (index%(int(len_dense/res_dense)+1))*res_dense;
      y = start_y + (index/(int(len_dense/res_dense)+1))*res_dense;
      p.x = x;
      p.y = y;
      p.z = dense_heightmap[index];
      dense_cloud.points.push_back(p);
      exteroception_msg.dense.push_back(p.z);
    }

    // 최종 결과를 publish
    // pub_->publish(dense_cloud);
    pub_sparse->publish(sparse_cloud);
    pub_dense->publish(dense_cloud);
    pub_exteroception->publish(exteroception_msg);

    // 시간 확인
    auto final_time = this->now();
    time_diff_sec = (final_time - first_time).seconds();
    RCLCPP_INFO(this->get_logger(), "Published legacy PointCloud with %lu points | Δt = %.3f Hz", sparse_cloud.points.size(), 1.0/time_diff_sec);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud2;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_sparse;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_dense;
  rclcpp::Publisher<rover_msgs::msg::Exteroception>::SharedPtr pub_exteroception;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudConverter>());
  rclcpp::shutdown();
  return 0;
}
