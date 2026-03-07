// yolov11n_rknn/src/yolo_detection_node.cpp
/**
 * @file yolo_detection_node.cpp
 * @brief YOLOv11n RKNN 检测ROS2节点
 *
 * 该节点从摄像头读取图像，使用RKNN加速的YOLOv11n模型进行目标检测，
 * 并将检测结果发布到ROS2话题。
 *
 * 发布的话题:
 *   /yolo/detections      - vision_msgs/Detection2DArray 检测结果
 *   /yolo/detection_image - sensor_msgs/Image 检测图像（可选）
 *
 * 参数:
 *   model_path        - RKNN模型文件路径（默认使用包内models目录）
 *   conf_threshold    - 置信度阈值 [0, 1]
 *   nms_threshold     - NMS IoU阈值 [0, 1]
 *   show_detection    - 是否显示检测窗口
 *   publish_image     - 是否发布检测图像
 *   camera_id         - 摄像头设备ID
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"
#include "cv_bridge/cv_bridge.h"
#include "yolov11n_rknn/yolo_detector.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <chrono>

namespace yolov11n_rknn {

/**
 * @brief YOLO检测ROS2节点
 *
 * 负责管理摄像头、运行检测、发布结果和可视化
 */
class YoloDetectionNode : public rclcpp::Node {
public:
    YoloDetectionNode() : rclcpp::Node("yolo_detection_node") {
        // 获取包的共享目录路径
        std::string package_share_dir;
        try {
            package_share_dir = ament_index_cpp::get_package_share_directory("yolov11n_rknn");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to find package 'yolov11n_rknn': %s", e.what());
            return;
        }

        // 设置默认模型路径（使用包内的models目录）
        std::string default_model_path = package_share_dir + "/models/light-1.rknn";

        // 声明参数
        declare_parameter("model_path", default_model_path);
        declare_parameter("conf_threshold", 0.5);
        declare_parameter("nms_threshold", 0.45);
        declare_parameter("show_detection", true);
        declare_parameter("publish_image", false);
        declare_parameter("camera_id", 0);
        declare_parameter("num_classes", 1);
        declare_parameter("class_names", std::vector<std::string>{"ball"});

        // 获取参数
        auto config = get_config_from_params();

        // 创建检测器
        detector_ = std::make_unique<YoloDetector>(config);

        // 初始化检测器
        if (!detector_->init()) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize YOLO detector");
            return;
        }

        // 创建发布者
        detection_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
            "/yolo/detections", 10);

        if (publish_image_) {
            image_pub_ = create_publisher<sensor_msgs::msg::Image>(
                "/yolo/detection_image", 10);
        }

        // 打开摄像头
        if (!open_camera()) {
            RCLCPP_ERROR(get_logger(), "Failed to open camera");
            return;
        }

        // 创建检测定时器
        const auto& detector_config = detector_->get_config();
        int period_ms = 1000 / detector_config.target_fps;
        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            [this] { detect_callback(); }
        );

        // 初始化FPS统计
        frame_count_ = 0;
        last_fps_time_ = now();
        current_fps_ = 0;

        // 打印配置信息
        log_config_info();

        RCLCPP_INFO(get_logger(), "YOLO Detection Node started successfully");
    }

    ~YoloDetectionNode() override {
        // 清理资源
        cv::destroyAllWindows();

        if (cap_.isOpened()) {
            cap_.release();
        }
    }

private:
    /**
     * @brief 从ROS参数获取检测器配置
     */
    DetectorConfig get_config_from_params() {
        DetectorConfig config;

        config.model_path = get_parameter("model_path").as_string();
        config.conf_threshold = get_parameter("conf_threshold").as_double();
        config.nms_threshold = get_parameter("nms_threshold").as_double();
        config.num_classes = get_parameter("num_classes").as_int();
        config.class_names = get_parameter("class_names").as_string_array();

        show_detection_ = get_parameter("show_detection").as_bool();
        publish_image_ = get_parameter("publish_image").as_bool();
        camera_id_ = get_parameter("camera_id").as_int();

        // 设置输入尺寸（固定为 640x640）
        config.input_width = 640;
        config.input_height = 640;

        return config;
    }

    /**
     * @brief 打开摄像头设备
     */
    bool open_camera() {
        cap_.open(camera_id_, cv::CAP_V4L2);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(get_logger(), "Failed to open camera device %d", camera_id_);
            return false;
        }

        // 配置摄像头参数 (使用640x480原生分辨率，最高120fps)
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        cap_.set(cv::CAP_PROP_FPS, 120);

        // 验证配置
        int actual_width = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
        int actual_height = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
        double actual_fps = cap_.get(cv::CAP_PROP_FPS);

        RCLCPP_INFO(get_logger(),
                    "Camera opened: %dx%d @ %.1f fps",
                    actual_width, actual_height, actual_fps);

        return true;
    }

    /**
     * @brief 打印配置信息
     */
    void log_config_info() {
        const auto& config = detector_->get_config();
        RCLCPP_INFO(get_logger(), "=== YOLO Detection Configuration ===");
        RCLCPP_INFO(get_logger(), "Model path: %s", config.model_path.c_str());
        RCLCPP_INFO(get_logger(), "Number of classes: %d", config.num_classes);
        RCLCPP_INFO(get_logger(), "Input resolution: %dx%d", config.input_width, config.input_height);
        RCLCPP_INFO(get_logger(), "Confidence threshold: %.2f", config.conf_threshold);
        RCLCPP_INFO(get_logger(), "NMS threshold: %.2f", config.nms_threshold);
        RCLCPP_INFO(get_logger(), "Show detection: %s", show_detection_ ? "YES" : "NO");
        RCLCPP_INFO(get_logger(), "Publish image: %s", publish_image_ ? "YES" : "NO");
        RCLCPP_INFO(get_logger(), "=====================================");

        // 打印无显示模式提示
        print_headless_mode_info();
    }

    /**
     * @brief 检测定时器回调函数
     */
    void detect_callback() {
        auto callback_start = std::chrono::high_resolution_clock::now();

        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                "Failed to grab frame from camera");
            return;
        }

        auto capture_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float, std::milli> capture_time = capture_end - callback_start;
        last_capture_time_ms_ = capture_time.count();

        // 更新FPS统计
        update_fps();

        // 运行检测
        std::vector<DetectBox> detections = detector_->detect(frame);
        last_detection_count_ = detections.size();

        // 发布检测结果
        publish_detections(detections, frame);

        // 可视化（降频显示以提升性能）
        if (show_detection_) {
            display_frame_count_++;
            if (display_frame_count_ >= 3) {  // 每 3 帧显示一次
                display_results(frame, detections);
                display_frame_count_ = 0;
            }
        }
    }

    /**
     * @brief 更新帧率统计
     */
    void update_fps() {
        frame_count_++;
        auto current_time = now();

        double elapsed = (current_time - last_fps_time_).seconds();
        if (elapsed >= 1.0) {
            current_fps_ = static_cast<int>(frame_count_ / elapsed);
            frame_count_ = 0;
            last_fps_time_ = current_time;

            // 如果没有显示窗口，在终端输出性能信息
            if (!show_detection_) {
                float capture_time = last_capture_time_ms_;
                float preprocess_time = detector_->get_last_preprocess_time_ms();
                float inference_time = detector_->get_last_inference_time_ms();
                float postprocess_time = detector_->get_last_postprocess_time_ms();
                float total_time = detector_->get_last_total_time_ms();

                RCLCPP_INFO(get_logger(),
                            "FPS: %d | Cap: %.1fms | Pre: %.1fms | Inf: %.1fms | Post: %.1fms | Detect: %.1fms | Det: %zu",
                            current_fps_, capture_time, preprocess_time, inference_time, postprocess_time, total_time,
                            last_detection_count_);
            }
        }
    }

    /**
     * @brief 打印无显示模式提示
     */
    void print_headless_mode_info() {
        if (!show_detection_ && !headless_info_printed_) {
            RCLCPP_INFO(get_logger(), "Running in headless mode (no display). Performance stats will be printed every second...");
            headless_info_printed_ = true;
        }
    }

    /**
     * @brief 发布检测结果到ROS话题
     */
    void publish_detections(const std::vector<DetectBox>& detections,
                           const cv::Mat& frame) {
        auto msg = vision_msgs::msg::Detection2DArray();
        msg.header.stamp = now();
        msg.header.frame_id = "camera";

        const auto& config = detector_->get_config();

        for (const auto& det : detections) {
            auto det_msg = vision_msgs::msg::Detection2D();
            det_msg.header = msg.header;

            // 设置类别信息
            auto hypothesis = vision_msgs::msg::ObjectHypothesisWithPose();

            if (det.class_id < static_cast<int>(config.class_names.size())) {
                hypothesis.hypothesis.class_id = config.class_names[det.class_id];
            } else {
                hypothesis.hypothesis.class_id = "class_" + std::to_string(det.class_id);
            }

            hypothesis.hypothesis.score = det.score;
            det_msg.results.push_back(hypothesis);

            // 设置边界框
            det_msg.bbox.center.position.x = det.center_x();
            det_msg.bbox.center.position.y = det.center_y();
            det_msg.bbox.size_x = det.width();
            det_msg.bbox.size_y = det.height();

            msg.detections.push_back(det_msg);
        }

        detection_pub_->publish(msg);

        // 可选：发布检测图像
        if (publish_image_ && !detections.empty()) {
            cv::Mat display = frame.clone();
            draw_detections(display, detections);
            publish_detection_image(display, msg.header);
        }
    }

    /**
     * @brief 发布检测图像
     */
    void publish_detection_image(const cv::Mat& image,
                                 const std_msgs::msg::Header& header) {
        cv_bridge::CvImage cv_img;
        cv_img.header = header;
        cv_img.encoding = sensor_msgs::image_encodings::BGR8;
        cv_img.image = image;

        image_pub_->publish(*cv_img.toImageMsg());
    }

    /**
     * @brief 在图像上绘制检测结果
     */
    void draw_detections(cv::Mat& image, const std::vector<DetectBox>& detections) {
        const auto& config = detector_->get_config();

        for (const auto& det : detections) {
            // 绘制边界框
            cv::Rect box = det.to_cv_rect();
            cv::rectangle(image, box, cv::Scalar(0, 255, 0), 2);

            // 绘制标签背景和文字
            std::string class_name = (det.class_id < static_cast<int>(config.class_names.size()))
                                     ? config.class_names[det.class_id]
                                     : "class_" + std::to_string(det.class_id);

            std::string label = cv::format("%s %.2f", class_name.c_str(), det.score);

            int baseline;
            cv::Size text_size = cv::getTextSize(
                label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);

            cv::Point text_origin(box.x, std::max(0, box.y - 5));

            // 标签背景
            cv::rectangle(image,
                         text_origin + cv::Point(0, baseline),
                         text_origin + cv::Point(text_size.width, -text_size.height),
                         cv::Scalar(0, 0, 255), cv::FILLED);

            // 标签文字
            cv::putText(image, label, text_origin,
                       cv::FONT_HERSHEY_SIMPLEX, 0.6,
                       cv::Scalar(255, 255, 255), 2);
        }
    }

    /**
     * @brief 显示检测结果窗口
     */
    void display_results(const cv::Mat& frame, const std::vector<DetectBox>& detections) {
        cv::Mat display = frame.clone();
        draw_detections(display, detections);

        // 绘制FPS信息
        std::string fps_text = cv::format("FPS: %d", current_fps_);
        cv::putText(display, fps_text, cv::Point(10, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

        // 绘制检测数量
        std::string count_text = cv::format("Detections: %zu", detections.size());
        cv::putText(display, count_text, cv::Point(10, 65),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        // 显示详细性能分析
        float preprocess_time = detector_->get_last_preprocess_time_ms();
        float inference_time = detector_->get_last_inference_time_ms();
        float postprocess_time = detector_->get_last_postprocess_time_ms();
        float total_time = detector_->get_last_total_time_ms();

        std::string time_text = cv::format("Pre: %.1f | Inf: %.1f | Post: %.1f | Total: %.1f ms",
                                           preprocess_time, inference_time, postprocess_time, total_time);
        cv::putText(display, time_text, cv::Point(10, 95),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

        cv::imshow("YOLO Detection", display);
        cv::waitKey(5);  // 增加 waitKey 时间，减少处理频率
    }

private:
    // ROS2 组件
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 核心组件
    std::unique_ptr<YoloDetector> detector_;
    cv::VideoCapture cap_;

    // 配置参数
    bool show_detection_;
    bool publish_image_;
    int camera_id_;

    // FPS统计
    int frame_count_;
    rclcpp::Time last_fps_time_;
    int current_fps_;

    // 显示降频计数器
    int display_frame_count_ = 0;

    // 最后一次检测数量（用于终端输出）
    size_t last_detection_count_ = 0;

    // 最后一次摄像头采集时间
    float last_capture_time_ms_ = 0.0f;

    // 是否已打印无显示模式提示
    bool headless_info_printed_ = false;
};

} // namespace yolov11n_rknn

//==============================================================================
// 主函数
//==============================================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<yolov11n_rknn::YoloDetectionNode>();

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"),
                     "Exception in spin: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
