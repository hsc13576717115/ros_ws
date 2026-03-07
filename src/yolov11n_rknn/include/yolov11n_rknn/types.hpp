// yolov11n_rknn/include/yolov11n_rknn/types.hpp
#pragma once

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

namespace yolov11n_rknn {

/**
 * @brief 检测框结构
 *
 * 表示单个目标检测结果
 */
struct DetectBox {
    int class_id = 0;      ///< 类别ID
    float score = 0.0f;    ///< 置信度分数 [0, 1]
    float xmin = 0.0f;     ///< 边界框左上角x坐标
    float ymin = 0.0f;     ///< 边界框左上角y坐标
    float xmax = 0.0f;     ///< 边界框右下角x坐标
    float ymax = 0.0f;     ///< 边界框右下角y坐标

    DetectBox() = default;

    DetectBox(int id, float s, float x1, float y1, float x2, float y2)
        : class_id(id), score(s), xmin(x1), ymin(y1), xmax(x2), ymax(y2) {}

    /**
     * @brief 获取边界框中心点x坐标
     */
    float center_x() const { return (xmin + xmax) * 0.5f; }

    /**
     * @brief 获取边界框中心点y坐标
     */
    float center_y() const { return (ymin + ymax) * 0.5f; }

    /**
     * @brief 获取边界框宽度
     */
    float width() const { return xmax - xmin; }

    /**
     * @brief 获取边界框高度
     */
    float height() const { return ymax - ymin; }

    /**
     * @brief 转换为OpenCV Rect
     */
    cv::Rect to_cv_rect() const {
        return cv::Rect(
            static_cast<int>(xmin),
            static_cast<int>(ymin),
            static_cast<int>(width()),
            static_cast<int>(height())
        );
    }

    /**
     * @brief 检查边界框是否有效
     */
    bool is_valid() const {
        return xmax > xmin && ymax > ymin && score > 0.0f;
    }
};

/**
 * @brief YOLOv11 模型输出配置
 *
 * 定义模型的输出特征图尺寸和步长（640x640 输入）
 */
struct ModelOutputConfig {
    static constexpr int NUM_HEADS = 3;                           ///< 检测头数量
    static constexpr int DFL_CHANNELS = 16;                       ///< DFL通道数
    static constexpr int NUM_OUTPUTS = 6;                        ///< RKNN输出数量

    // 各检测头输出尺寸 [height, width] (对于 640x640 输入)
    // P3: 640/8 = 80x80
    // P4: 640/16 = 40x40
    // P5: 640/32 = 20x20
    static const std::vector<std::vector<int>> MAP_SIZES;

    // 各检测头步长
    static const std::vector<int> STRIDES;
};

/**
 * @brief YOLO 检测器配置参数
 */
struct DetectorConfig {
    // 模型配置
    std::string model_path;           ///< RKNN模型路径
    int num_classes = 1;              ///< 类别数量
    std::vector<std::string> class_names;  ///< 类别名称列表

    // 输入配置 (640x640)
    int input_width = 640;            ///< 模型输入宽度
    int input_height = 640;           ///< 模型输入高度

    // 检测阈值
    float conf_threshold = 0.5f;      ///< 置信度阈值
    float nms_threshold = 0.45f;      ///< NMS IoU阈值

    // 性能配置
    int target_fps = 60;              ///< 目标帧率

    DetectorConfig() = default;

    /**
     * @brief 验证配置参数的有效性
     * @return true 如果配置有效
     */
    bool is_valid() const {
        return !model_path.empty() &&
               input_width > 0 && input_height > 0 &&
               num_classes > 0 &&
               conf_threshold > 0.0f && conf_threshold <= 1.0f &&
               nms_threshold > 0.0f && nms_threshold <= 1.0f &&
               target_fps > 0;
    }
};

} // namespace yolov11n_rknn
