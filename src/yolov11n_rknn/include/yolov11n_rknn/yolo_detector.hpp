// yolov11n_rknn/include/yolov11n_rknn/yolo_detector.hpp
#pragma once

#include "yolov11n_rknn/types.hpp"
#include <rknn_api.h>
#include <memory>
#include <string>

namespace yolov11n_rknn {

/**
 * @brief YOLOv11n RKNN 检测器类
 *
 * 负责加载RKNN模型并进行目标检测推理
 *
 * 使用示例:
 * @code
 *   DetectorConfig config;
 *   config.model_path = "/path/to/model.rknn";
 *   config.num_classes = 80;
 *   config.class_names = {"person", "car", ...};
 *
 *   YoloDetector detector(config);
 *   if (!detector.init()) {
 *     // 处理初始化失败
 *   }
 *
 *   cv::Mat image = cv::imread("test.jpg");
 *   auto detections = detector.detect(image);
 * @endcode
 */
class YoloDetector {
public:
    /**
     * @brief 构造函数
     * @param config 检测器配置
     */
    explicit YoloDetector(const DetectorConfig& config);

    /**
     * @brief 析构函数，自动释放RKNN资源
     */
    ~YoloDetector();

    /**
     * @brief 初始化RKNN模型
     * @return true 初始化成功，false 失败
     */
    bool init();

    /**
     * @brief 对图像进行目标检测
     * @param image 输入图像（BGR格式）
     * @return 检测结果列表
     */
    std::vector<DetectBox> detect(const cv::Mat& image);

    /**
     * @brief 检查检测器是否已初始化
     * @return true 已初始化
     */
    bool is_initialized() const { return ctx_ != 0; }

    /**
     * @brief 获取配置
     */
    const DetectorConfig& get_config() const { return config_; }

    /**
     * @brief 获取最后一次推理时间（毫秒）
     */
    float get_last_inference_time_ms() const { return last_inference_time_ms_; }

    /**
     * @brief 获取最后一次预处理时间（毫秒）
     */
    float get_last_preprocess_time_ms() const { return last_preprocess_time_ms_; }

    /**
     * @brief 获取最后一次后处理时间（毫秒）
     */
    float get_last_postprocess_time_ms() const { return last_postprocess_time_ms_; }

    /**
     * @brief 获取最后一次总时间（毫秒）
     */
    float get_last_total_time_ms() const { return last_total_time_ms_; }

private:
    /**
     * @brief 加载RKNN模型文件
     * @param path 模型文件路径
     * @param out_size 输出模型大小
     * @return 模型数据指针，需调用者释放
     */
    unsigned char* load_model_file(const std::string& path, int* out_size);

    /**
     * @brief 预处理图像：调整大小和颜色空间转换
     * @param image 输入图像
     * @return 处理后的RGB图像
     */
    cv::Mat preprocess(const cv::Mat& image);

    /**
     * @brief 后处理：解码模型输出并应用NMS
     * @param outputs RKNN输出张量
     * @param orig_h 原始图像高度
     * @param orig_w 原始图像宽度
     * @return 检测结果列表
     */
    std::vector<DetectBox> postprocess(const std::vector<rknn_output>& outputs,
                                       int orig_h, int orig_w);

    /**
     * @brief DFL (Distribution Focal Loss) 解码
     *
     * YOLOv11使用DFL来预测边界框坐标，将坐标建模为16个bin的概率分布
     *
     * @param reg 回归输出张量
     * @param map_h 特征图高度
     * @param map_w 特征图宽度
     * @param h 像素y坐标
     * @param w 像素x坐标
     * @param out_regdfl 输出4个边界值 [top, left, bottom, right]
     */
    void decode_dfl(const std::vector<float>& reg, int map_h, int map_w,
                    int h, int w, float out_regdfl[4]);

    /**
     * @brief 应用非极大值抑制 (NMS)
     * @param detections 原始检测结果
     * @param iou_threshold IoU阈值
     * @return NMS后的检测结果
     */
    std::vector<DetectBox> apply_nms(const std::vector<DetectBox>& detections,
                                     float iou_threshold);

    /**
     * @brief 计算两个矩形的IoU
     */
    float calculate_iou(const cv::Rect& a, const cv::Rect& b) const;

    /**
     * @brief Sigmoid激活函数
     */
    static float sigmoid(float x) {
        return 1.0f / (1.0f + std::exp(-x));
    }

private:
    DetectorConfig config_;       ///< 检测器配置
    rknn_context ctx_ = 0;  ///< RKNN上下文
    float last_preprocess_time_ms_ = 0.0f;  ///< 最后一次预处理时间
    float last_inference_time_ms_ = 0.0f;  ///< 最后一次推理时间
    float last_postprocess_time_ms_ = 0.0f;  ///< 最后一次后处理时间
    float last_total_time_ms_ = 0.0f;  ///< 最后一次总时间
};

} // namespace yolov11n_rknn
