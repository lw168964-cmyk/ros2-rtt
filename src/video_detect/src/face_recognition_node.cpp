// C++标准库
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// ROS2核心依赖
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

// 人脸检测参数常量（可根据需求调整）
const double SCALE_FACTOR = 1.1;    // 图像缩放比例
const int MIN_NEIGHBORS = 5;        // 候选矩形邻居数量
const cv::Size MIN_FACE_SIZE(30, 30); // 最小检测人脸尺寸

// ==============================================
// 【独立的人脸检测器类】
// 职责：仅负责人脸检测算法的实现
// 特点：完全独立于ROS2，可在任何OpenCV项目中直接复用
// ==============================================
class HaarFaceDetector
{
public:
  // 构造函数：初始化检测器并加载模型
  // 参数：cascade_path - Haar分类器模型文件的绝对路径
  explicit HaarFaceDetector(const std::string & cascade_path)
  {
    // 加载人脸分类器模型
    is_initialized_ = cascade_.load(cascade_path);

    // 设置默认检测参数
    scale_factor_ = SCALE_FACTOR;
    min_neighbors_ = MIN_NEIGHBORS;
    min_face_size_ = MIN_FACE_SIZE;
  }

  // 析构函数
  ~HaarFaceDetector() = default;

  // 核心检测方法：输入图像，输出检测到的人脸矩形列表
  // 参数：frame - 输入的BGR彩色图像
  // 返回值：检测到的所有人脸矩形
  std::vector<cv::Rect> detect(const cv::Mat & frame)
  {
    std::vector<cv::Rect> faces;

    // 如果检测器未初始化成功，直接返回空结果
    if (!is_initialized_) {
      return faces;
    }

    // 1. 转换为灰度图（Haar检测必须在灰度图上进行）
    cv::Mat gray_frame;
    cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);

    // 2. 直方图均衡化，提高低光照下的检测效果
    cv::equalizeHist(gray_frame, gray_frame);

    // 3. 执行多尺度人脸检测
    cascade_.detectMultiScale(
      gray_frame,
      faces,
      scale_factor_,
      min_neighbors_,
      0,
      min_face_size_
    );

    return faces;
  }

  // ==============================================
  // 参数配置方法（运行时动态调整检测参数）
  // ==============================================
  void set_scale_factor(double scale_factor) {scale_factor_ = scale_factor;}
  void set_min_neighbors(int min_neighbors) {min_neighbors_ = min_neighbors;}
  void set_min_face_size(const cv::Size & size) {min_face_size_ = size;}

  // 获取检测器初始化状态
  bool is_initialized() const {return is_initialized_;}

private:
  cv::CascadeClassifier cascade_;    // Haar级联分类器
  bool is_initialized_;              // 检测器初始化状态标志

  // 检测参数（私有成员，通过setter方法配置）
  double scale_factor_;
  int min_neighbors_;
  cv::Size min_face_size_;
};

// ==============================================
// 【ROS2节点类】
// 职责：仅负责ROS通信、图像流转和结果显示
// 特点：不包含任何算法逻辑，仅作为算法与ROS系统的桥梁
// ==============================================
class FaceDetectionNode : public rclcpp::Node
{
public:
  // 构造函数：初始化ROS节点和人脸检测器
  // 参数：node_name - ROS节点名称
  //       cascade_path - 人脸分类器模型路径
  FaceDetectionNode(const std::string & node_name, const std::string & default_cascade_path)
  : Node(node_name)
  {
    RCLCPP_INFO(this->get_logger(), "人脸检测节点启动中...");

    image_topic_ = this->declare_parameter<std::string>(
      "image_topic", "/camera/color/image_raw");
    const std::string cascade_path = this->declare_parameter<std::string>(
      "cascade_path", default_cascade_path);
    window_name_ = this->declare_parameter<std::string>(
      "window_name", "Astra Pro Face Detection");

    // 1. 初始化人脸检测器（组合关系：节点包含检测器）
    face_detector_ = std::make_unique<HaarFaceDetector>(cascade_path);

    // 2. 检查检测器是否初始化成功
    if (!face_detector_->is_initialized()) {
      RCLCPP_ERROR(this->get_logger(), "人脸检测器初始化失败！");
      RCLCPP_ERROR(this->get_logger(), "请检查分类器路径: %s", cascade_path.c_str());
      throw std::runtime_error("failed to initialize face detector");
    }
    RCLCPP_INFO(this->get_logger(), "人脸检测器初始化成功");

    // 3. 可选：运行时动态调整检测参数
    face_detector_->set_scale_factor(1.1);
    face_detector_->set_min_neighbors(5);
    face_detector_->set_min_face_size(cv::Size(30, 30));

    // 4. 创建图像订阅者
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&FaceDetectionNode::image_callback, this, std::placeholders::_1)
    );

    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);

    RCLCPP_INFO(this->get_logger(), "正在订阅官方图像话题: %s", image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "节点启动完成，正在等待图像...");
  }

  // 析构函数：释放资源
  ~FaceDetectionNode() override
  {
    cv::destroyAllWindows();
    RCLCPP_INFO(this->get_logger(), "人脸检测节点已退出");
  }

private:
  // 图像回调函数：收到图像后自动执行
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      // 1. ROS图像消息转换为OpenCV格式
      cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

      // 2. 调用人脸检测器进行检测（核心业务逻辑）
      std::vector<cv::Rect> faces = face_detector_->detect(frame);

      // 3. 绘制检测结果
      draw_detections(frame, faces);

      // 4. 显示结果
      cv::imshow(window_name_, frame);
      cv::waitKey(1);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "图像转换失败: %s", e.what());
    }
  }

  // 绘制检测结果的辅助方法
  void draw_detections(cv::Mat & frame, const std::vector<cv::Rect> & faces)
  {
    // 绘制每个人脸的矩形框和标签
    for (const cv::Rect & face : faces) {
      // 绘制蓝色矩形框
      cv::rectangle(
        frame,
        face.tl(),
        face.br(),
        cv::Scalar(255, 0, 0),
        2
      );

      // 绘制人脸标签
      cv::putText(
        frame,
        "Face",
        cv::Point(face.x, face.y - 10),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(255, 0, 0),
        1
      );
    }

    // 绘制检测到的人脸总数
    cv::putText(
      frame,
      "Detected: " + std::to_string(faces.size()),
      cv::Point(10, 30),
      cv::FONT_HERSHEY_SIMPLEX,
      1.0,
      cv::Scalar(0, 255, 0),
      2
    );
  }

  // 成员变量
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  std::unique_ptr<HaarFaceDetector> face_detector_;   // 人脸检测器智能指针
  std::string image_topic_;
  std::string window_name_;
};

// ==============================================
// 主函数：程序入口
// ==============================================
int main(int argc, char * argv[])
{
  // 1. 初始化ROS2环境
  rclcpp::init(argc, argv);

  // 2. 配置参数
  const std::string node_name = "face_detection_node";
  // 【⚠️ 请修改为你系统的实际分类器路径】
  const std::string cascade_path =
    "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml";

  // 3. 创建并运行节点
  try {
    auto node = std::make_shared<FaceDetectionNode>(node_name, cascade_path);
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger(node_name), "节点启动失败: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  // 4. 清理资源并退出
  // node->destroy_node();
  rclcpp::shutdown();

  return 0;
}
