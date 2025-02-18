#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <tf2_ros/transform_broadcaster.h>

#include "sv/dsol/extra.h"
#include "sv/dsol/node_util.h"
#include "sv/ros1/msg_conv.h"
#include "sv/util/dataset.h"
#include "sv/util/logging.h"
#include "sv/util/ocv.h"

namespace sv::dsol {

using SE3d = Sophus::SE3d;
namespace gm = geometry_msgs;
namespace sm = sensor_msgs;
namespace vm = visualization_msgs;

struct NodeData {
  explicit NodeData(const ros::NodeHandle& pnh);

  void InitOdom();
  void InitRosIO();
  void InitDataset();

  void PublishOdom(const std_msgs::Header& header, const Sophus::SE3d& Twc);
  void PublishCloud(const std_msgs::Header& header) const;
  void SendTransform(const gm::PoseStamped& pose_msg,
                     const std::string& child_frame);
  void Run();

  bool reverse_{false};
  double freq_{10.0};
  double data_max_depth_{0};
  double cloud_max_depth_{100};
  cv::Range data_range_{0, 0};

  Dataset dataset_;
  MotionModel motion_;
  TumFormatWriter writer_;
  DirectOdometry odom_;

  KeyControl ctrl_;
  std::string frame_{"fixed"};
  tf2_ros::TransformBroadcaster tfbr_;

  ros::NodeHandle pnh_;
  ros::Publisher clock_pub_;
  ros::Publisher pose_array_pub_;
  ros::Publisher align_marker_pub_;
  PosePathPublisher gt_pub_;
  PosePathPublisher kf_pub_;
  PosePathPublisher odom_pub_;

  ros::Publisher points_pub_;
};

NodeData::NodeData(const ros::NodeHandle& pnh) : pnh_{pnh} {
  InitRosIO();
  InitDataset();
  InitOdom();

  const int wait_ms = pnh_.param<int>("wait_ms", 0);
  ROS_INFO_STREAM("wait_ms: " << wait_ms);
  ctrl_ = KeyControl(wait_ms);

  const auto save = pnh_.param<std::string>("save", "");
  writer_ = TumFormatWriter(save);
  if (!writer_.IsDummy()) {
    ROS_WARN_STREAM("Writing results to: " << writer_.filename());
  }

  const auto alpha = pnh_.param<double>("motion_alpha", 0.5);
  motion_ = MotionModel(alpha);
  ROS_INFO_STREAM("motion_alpha: " << motion_.alpha());

  // this is to make camera z pointing forward
  //  const Eigen::Quaterniond q_f_c0(
  //      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
  //      Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ()));
  //  T_f_c0_.setQuaternion(q_f_c0);
  //  ROS_INFO_STREAM("T_f_c0: \n" << T_f_c0_.matrix());
}

void NodeData::InitRosIO() {
  clock_pub_ = pnh_.advertise<rosgraph_msgs::Clock>("/clock", 1);

  gt_pub_ = PosePathPublisher(pnh_, "gt", frame_);
  kf_pub_ = PosePathPublisher(pnh_, "kf", frame_);
  odom_pub_ = PosePathPublisher(pnh_, "odom", frame_);
  points_pub_ = pnh_.advertise<sm::PointCloud2>("points", 1);
  pose_array_pub_ = pnh_.advertise<gm::PoseArray>("poses", 1);
  align_marker_pub_ = pnh_.advertise<vm::Marker>("align_graph", 1);
}

void NodeData::InitDataset() {
  const auto data_dir = pnh_.param<std::string>("data_dir", {});
  dataset_ = CreateDataset(data_dir);
  if (!dataset_) {
    ROS_ERROR_STREAM("Invalid dataset at: " << data_dir);
    ros::shutdown();
  }
  ROS_INFO_STREAM(dataset_.Repr());

  pnh_.getParam("start", data_range_.start);
  pnh_.getParam("end", data_range_.end);
  pnh_.getParam("reverse", reverse_);

  if (data_range_.end <= 0) {
    data_range_.end += dataset_.size();
  }
  ROS_INFO("Data range: [%d, %d)", data_range_.start, data_range_.end);
  ROS_INFO("Reverse: %s", reverse_ ? "true" : "false");

  pnh_.getParam("freq", freq_);
  pnh_.getParam("data_max_depth", data_max_depth_);
  pnh_.getParam("cloud_max_depth", cloud_max_depth_);

  ROS_INFO_STREAM("Freq: " << freq_);
  ROS_INFO_STREAM("Max depth: " << data_max_depth_);
}

void NodeData::InitOdom() {
  {
    auto cfg = ReadOdomCfg({pnh_, "odom"});
    pnh_.getParam("tbb", cfg.tbb);
    pnh_.getParam("log", cfg.log);
    pnh_.getParam("vis", cfg.vis);
    odom_.Init(cfg);
  }
  odom_.selector = PixelSelector(ReadSelectCfg({pnh_, "select"}));
  odom_.matcher = StereoMatcher(ReadStereoCfg({pnh_, "stereo"}));
  odom_.aligner = FrameAligner(ReadDirectCfg({pnh_, "align"}));
  odom_.adjuster = BundleAdjuster(ReadDirectCfg({pnh_, "adjust"}));
  odom_.cmap = GetColorMap(pnh_.param<std::string>("cm", "jet"));

  ROS_INFO_STREAM(odom_.Repr());
}

void NodeData::PublishCloud(const std_msgs::Header& header) const {
  if (points_pub_.getNumSubscribers() == 0) return;

  static sensor_msgs::PointCloud2 cloud;
  cloud.header = header;
  cloud.point_step = 16;
  cloud.fields = MakePointFields("xyzi");

  ROS_DEBUG_STREAM(odom_.window.MargKf().status().Repr());
  Keyframe2Cloud(odom_.window.MargKf(), cloud, cloud_max_depth_);
  points_pub_.publish(cloud);
}

void NodeData::SendTransform(const geometry_msgs::PoseStamped& pose_msg,
                             const std::string& child_frame) {
  gm::TransformStamped tf_msg;
  tf_msg.header = pose_msg.header;
  tf_msg.child_frame_id = child_frame;
  Ros2Ros(pose_msg.pose, tf_msg.transform);
  tfbr_.sendTransform(tf_msg);
}

void NodeData::Run() {
  ros::Time time{};
  const auto dt = 1.0 / freq_;  // 设置时间步长
  const ros::Duration dtime{ros::Rate{freq_}};  // 设置时间间隔

  bool init_tf{false};  // 初始化标志
  SE3d T_c0_w_gt;  // 世界坐标系到相机0坐标系的初始变换
  SE3d dT_pred;  // 预测的变换

  int start_ind = reverse_ ? data_range_.end - 1 : data_range_.start;  // 设置起始索引
  int end_ind = reverse_ ? data_range_.start - 1 : data_range_.end;  // 设置结束索引
  const int delta = reverse_ ? -1 : 1;  // 设置步长

  // Marker
  vm::Marker align_marker;  // 对齐标记

  for (int ind = start_ind, cnt = 0; ind != end_ind; ind += delta, ++cnt) {
    if (!ros::ok() || !ctrl_.Wait()) break;  // 检查ROS状态和控制信号

    ROS_INFO("=== %d ===", ind);  // 输出当前索引
    rosgraph_msgs::Clock clock;
    clock.clock = time;
    clock_pub_.publish(clock);  // 发布时间戳

    // Image
    auto image_l = dataset_.Get(DataType::kImage, ind, 0);  // 获取左图像
    auto image_r = dataset_.Get(DataType::kImage, ind, 1);  // 获取右图像

    // Intrin
    if (!odom_.camera.Ok()) {
      const auto intrin = dataset_.Get(DataType::kIntrin, ind);  // 获取内参
      const auto camera = Camera::FromMat({image_l.cols, image_l.rows}, intrin);  // 创建相机模型
      odom_.SetCamera(camera);  // 设置相机模型
      ROS_INFO_STREAM(camera);  // 输出相机信息
    }

    // Depth
    auto depth = dataset_.Get(DataType::kDepth, ind, 0);  // 获取深度图

    if (!depth.empty()) {
      if (data_max_depth_ > 0) {
        cv::threshold(depth, depth, data_max_depth_, 0, cv::THRESH_TOZERO_INV);  // 处理深度图
      }
    }

    // Pose
    const auto pose_gt = dataset_.Get(DataType::kPose, ind, 0);  // 获取真实姿态

    // Record the inverse of the first transform
    if (!init_tf) {
      T_c0_w_gt = SE3dFromMat(pose_gt).inverse();  // 记录初始变换的逆
      ROS_INFO_STREAM("T_c0_w:\n" << T_c0_w_gt.matrix());  // 输出初始变换
      init_tf = true;  // 设置初始化标志
    }

    // Then we transform everything into c0 frame
    const auto T_c0_c_gt = T_c0_w_gt * SE3dFromMat(pose_gt);  // 将所有数据转换到相机0坐标系

    // Motion model predict
    if (!motion_.Ok()) {
      motion_.Init(T_c0_c_gt);  // 初始化运动模型
    } else {
      dT_pred = motion_.PredictDelta(dt);  // 预测变换
    }

    const auto T_pred = odom_.frame.Twc() * dT_pred;  // 计算预测姿态

    // Odom
    const auto status = odom_.Estimate(image_l, image_r, dT_pred, depth);  // 估计姿态
    ROS_INFO_STREAM(status.Repr());  // 输出估计状态

    // Motion model correct if tracking is ok and not first frame
    if (status.track.ok && ind != start_ind) {
      motion_.Correct(status.Twc(), dt);  // 校正运动模型
    } else {
      ROS_WARN_STREAM("Tracking failed (or 1st frame), slow motion model");  // 跟踪失败时警告
      motion_.Scale(0.5);  // 减慢运动模型
    }

    // Write to output
    writer_.Write(cnt, status.Twc());  // 写入输出

    ROS_DEBUG_STREAM("trans gt:   " << T_c0_c_gt.translation().transpose());  // 输出真实变换
    ROS_DEBUG_STREAM("trans pred: " << T_pred.translation().transpose());  // 输出预测变换
    ROS_DEBUG_STREAM("trans odom: " << status.Twc().translation().transpose());  // 输出估计变换
    ROS_DEBUG_STREAM("trans ba:   " << odom_.window.CurrKf().Twc().translation().transpose());  // 输出当前关键帧变换
    ROS_DEBUG_STREAM("aff_l: " << odom_.frame.state().affine_l.ab.transpose());  // 输出左仿射变换
    ROS_DEBUG_STREAM("aff_r: " << odom_.frame.state().affine_r.ab.transpose());  // 输出右仿射变换

    // publish stuff
    std_msgs::Header header;
    header.frame_id = frame_;
    header.stamp = time;

    gt_pub_.Publish(time, T_c0_c_gt);  // 发布真实姿态
    PublishOdom(header, status.Twc());  // 发布估计姿态

    if (status.map.remove_kf) {
      PublishCloud(header);  // 发布点云
    }

    // Draw align graph
    //    align_marker.header = header;
    //    DrawAlignGraph(status.Twc().translation(),
    //                   odom_.window.GetAllTrans(),
    //                   odom_.aligner.num_tracks(),
    //                   CV_RGB(1.0, 0.0, 0.0),
    //                   0.1,
    //                   align_marker);
    //    align_marker_pub_.publish(align_marker);

    time += dtime;  // 更新时间
  }
}

void NodeData::PublishOdom(const std_msgs::Header& header,
                           const Sophus::SE3d& Twc) {
  const auto odom_pose_msg = odom_pub_.Publish(header.stamp, Twc);
  SendTransform(odom_pose_msg, "camera");

  const auto poses = odom_.window.GetAllPoses();
  gm::PoseArray pose_array_msg;
  pose_array_msg.header = header;
  pose_array_msg.poses.resize(poses.size());
  for (size_t i = 0; i < poses.size(); ++i) {
    Sophus2Ros(poses.at(i), pose_array_msg.poses.at(i));
  }
  pose_array_pub_.publish(pose_array_msg);
}

}  // namespace sv::dsol

int main(int argc, char** argv) {
  ros::init(argc, argv, "dsol_processor");
  sv::dsol::NodeData node{ros::NodeHandle{"~"}};
  node.Run();
}
