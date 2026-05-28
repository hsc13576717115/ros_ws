// yolov11n_rknn/src/yolo_detector.cpp
#include "yolov11n_rknn/yolo_detector.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <algorithm>
#include <cstring>

namespace yolov11n_rknn {

// 定义静态常量 (640x640 输入对应的特征图尺寸)
const std::vector<std::vector<int>> ModelOutputConfig::MAP_SIZES = {
    {80, 80},   // P3: 小目标检测 (640/8 = 80)
    {40, 40},   // P4: 中等目标检测 (640/16 = 40)
    {20, 20}    // P5: 大目标检测 (640/32 = 20)
};

const std::vector<int> ModelOutputConfig::STRIDES = {8, 16, 32};

//==============================================================================
// YoloDetector 实现
//==============================================================================

YoloDetector::YoloDetector(const DetectorConfig& config)
    : config_(config) {
}

YoloDetector::~YoloDetector() {
    if (ctx_ != 0) {
        rknn_destroy(ctx_);
        ctx_ = 0;
    }
}

bool YoloDetector::init() {
    // 验证配置
    if (!config_.is_valid()) {
        RCLCPP_ERROR(rclcpp::get_logger("yolo_detector"),
                     "Invalid detector configuration");
        return false;
    }

    // 加载模型文件
    int model_size = 0;
    unsigned char* model_data = load_model_file(config_.model_path, &model_size);
    if (!model_data) {
        RCLCPP_ERROR(rclcpp::get_logger("yolo_detector"),
                     "Failed to load model from: %s", config_.model_path.c_str());
        return false;
    }

    // 初始化RKNN
    int ret = rknn_init(&ctx_, model_data, model_size,
                        RKNN_FLAG_PRIOR_MEDIUM, 0);
    free(model_data);

    if (ret != RKNN_SUCC) {
        RCLCPP_ERROR(rclcpp::get_logger("yolo_detector"),
                     "rknn_init failed with error code: %d", ret);
        return false;
    }

    // 查询SDK版本
    rknn_sdk_version version;
    ret = rknn_query(ctx_, RKNN_QUERY_SDK_VERSION, &version, sizeof(version));
    if (ret == RKNN_SUCC) {
        RCLCPP_INFO(rclcpp::get_logger("yolo_detector"),
                    "RKNN SDK version: %s", version.api_version);
    }

    RCLCPP_INFO(rclcpp::get_logger("yolo_detector"),
                "YOLO detector initialized with %d classes (input: %dx%d)",
                config_.num_classes, config_.input_width, config_.input_height);
    return true;
}

std::vector<DetectBox> YoloDetector::detect(const cv::Mat& image) {
    if (!is_initialized() || image.empty()) {
        return {};
    }

    auto total_start = std::chrono::high_resolution_clock::now();

    // 1. 预处理
    auto preprocess_start = std::chrono::high_resolution_clock::now();
    cv::Mat processed = preprocess(image);
    auto preprocess_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float, std::milli> preprocess_time = preprocess_end - preprocess_start;

    // 2. 准备RKNN输入
    rknn_input inputs[1] = {};
    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].buf = processed.data;
    inputs[0].size = processed.total() * processed.elemSize();

    std::vector<DetectBox> detections;

    // 3. 运行推理
    auto inference_start = std::chrono::high_resolution_clock::now();
    if (rknn_inputs_set(ctx_, 1, inputs) != RKNN_SUCC) {
        RCLCPP_ERROR(rclcpp::get_logger("yolo_detector"),
                     "Failed to set RKNN inputs");
        return {};
    }

    if (rknn_run(ctx_, 0) != RKNN_SUCC) {
        RCLCPP_ERROR(rclcpp::get_logger("yolo_detector"),
                     "Failed to run RKNN inference");
        return {};
    }
    auto inference_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float, std::milli> inference_time = inference_end - inference_start;

    // 4. 获取输出
    rknn_output outputs[ModelOutputConfig::NUM_OUTPUTS] = {};
    for (int i = 0; i < ModelOutputConfig::NUM_OUTPUTS; ++i) {
        outputs[i].want_float = 1;
    }

    if (rknn_outputs_get(ctx_, ModelOutputConfig::NUM_OUTPUTS,
                        outputs, 0) != RKNN_SUCC) {
        RCLCPP_ERROR(rclcpp::get_logger("yolo_detector"),
                     "Failed to get RKNN outputs");
        return {};
    }

    // 5. 后处理
    auto postprocess_start = std::chrono::high_resolution_clock::now();
    std::vector<rknn_output> output_vec(outputs, outputs + ModelOutputConfig::NUM_OUTPUTS);
    detections = postprocess(output_vec, image.rows, image.cols);
    auto postprocess_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float, std::milli> postprocess_time = postprocess_end - postprocess_start;

    // 记录时间
    auto total_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float, std::milli> total_time = total_end - total_start;

    last_preprocess_time_ms_ = preprocess_time.count();
    last_inference_time_ms_ = inference_time.count();
    last_postprocess_time_ms_ = postprocess_time.count();
    last_total_time_ms_ = total_time.count();

    rknn_outputs_release(ctx_, ModelOutputConfig::NUM_OUTPUTS, outputs);

    return detections;
}

//------------------------------------------------------------------------------

unsigned char* YoloDetector::load_model_file(const std::string& path, int* out_size) {
    FILE* fp = fopen(path.c_str(), "rb");
    if (!fp) {
        RCLCPP_ERROR(rclcpp::get_logger("yolo_detector"),
                     "Cannot open model file: %s (%s)",
                     path.c_str(), strerror(errno));
        return 0;
    }

    // 获取文件大小
    fseek(fp, 0, SEEK_END);
    int size = static_cast<int>(ftell(fp));
    fseek(fp, 0, SEEK_SET);

    // 分配内存并读取
    unsigned char* buffer = static_cast<unsigned char*>(malloc(size));
    if (!buffer) {
        RCLCPP_ERROR(rclcpp::get_logger("yolo_detector"),
                     "Failed to allocate %d bytes for model", size);
        fclose(fp);
        return 0;
    }

    size_t read_size = fread(buffer, 1, size, fp);
    fclose(fp);

    if (read_size != static_cast<size_t>(size)) {
        RCLCPP_ERROR(rclcpp::get_logger("yolo_detector"),
                     "Failed to read complete model file (got %zu of %d bytes)",
                     read_size, size);
        free(buffer);
        return 0;
    }

    *out_size = size;
    return buffer;
}

cv::Mat YoloDetector::preprocess(const cv::Mat& image) {
    cv::Mat resized;
    cv::resize(image, resized,
              cv::Size(config_.input_width, config_.input_height));

    cv::Mat rgb;
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
    return rgb;
}

void YoloDetector::decode_dfl(const std::vector<float>& reg,
                              int map_h, int map_w,
                              int h, int w, float out_regdfl[4]) {
    // DFL解码：每个边界框边由16个bin的概率分布表示
    // 通过期望值计算连续的边界位置
    for (int side = 0; side < 4; ++side) {
        int side_base = side * (ModelOutputConfig::DFL_CHANNELS * map_h * map_w);

        // 计算softmax: exp(x) / sum(exp(x))
        float sum_exp = 0.0f;
        float exps[ModelOutputConfig::DFL_CHANNELS];

        for (int k = 0; k < ModelOutputConfig::DFL_CHANNELS; ++k) {
            int idx = side_base + k * (map_h * map_w) + h * map_w + w;
            float v = reg[idx];
            float e = std::exp(v);
            exps[k] = e;
            sum_exp += e;
        }

        // 防止除零
        if (sum_exp <= 0.0f) {
            sum_exp = 1e-6f;
        }

        // 计算期望位置: sum(k * p_k)
        float location = 0.0f;
        for (int k = 0; k < ModelOutputConfig::DFL_CHANNELS; ++k) {
            location += exps[k] * k;
        }
        location /= sum_exp;

        out_regdfl[side] = location;
    }
}

std::vector<DetectBox> YoloDetector::postprocess(const std::vector<rknn_output>& outputs,
                                                 int orig_h, int orig_w) {
    // 转换输出为float向量
    std::vector<std::vector<float>> output_tensors;
    output_tensors.reserve(outputs.size());

    for (const auto& out : outputs) {
        size_t count = out.size / sizeof(float);
        float* data = static_cast<float*>(out.buf);
        output_tensors.emplace_back(data, data + count);
    }

    // 计算缩放比例
    float scale_h = static_cast<float>(orig_h) / config_.input_height;
    float scale_w = static_cast<float>(orig_w) / config_.input_width;

    std::vector<DetectBox> detections;

    // 遍历每个检测头
    for (int head_idx = 0; head_idx < ModelOutputConfig::NUM_HEADS; ++head_idx) {
        int stride = ModelOutputConfig::STRIDES[head_idx];
        int map_h = config_.input_height / stride;
        int map_w = config_.input_width / stride;

        const std::vector<float>& reg = output_tensors[head_idx * 2];
        const std::vector<float>& cls = output_tensors[head_idx * 2 + 1];

        // 验证输出尺寸
        size_t expected_cls_size = config_.num_classes * map_h * map_w;
        size_t expected_reg_size = 4 * ModelOutputConfig::DFL_CHANNELS * map_h * map_w;

        if (cls.size() != expected_cls_size || reg.size() != expected_reg_size) {
            RCLCPP_WARN(rclcpp::get_logger("yolo_detector"),
                        "Head %d: Unexpected output sizes (cls=%zu, reg=%zu)",
                        head_idx, cls.size(), reg.size());
            continue;
        }

        // 遍历特征图每个位置
        for (int h = 0; h < map_h; ++h) {
            for (int w = 0; w < map_w; ++w) {
                // 获取类别得分（取最高类别）
                float max_conf = 0.0f;
                int max_class_id = 0;

                for (int c = 0; c < config_.num_classes; ++c) {
                    int cls_idx = c * map_h * map_w + h * map_w + w;
                    float conf = sigmoid(cls[cls_idx]);
                    if (conf > max_conf) {
                        max_conf = conf;
                        max_class_id = c;
                    }
                }

                // 置信度过滤
                if (max_conf <= config_.conf_threshold) {
                    continue;
                }

                // DFL解码边界框
                float regdfl[4];
                decode_dfl(reg, map_h, map_w, h, w, regdfl);

                // 计算边界框坐标
                float grid_x = (w + 0.5f);
                float grid_y = (h + 0.5f);

                float x1 = (grid_x - regdfl[0]) * stride;
                float y1 = (grid_y - regdfl[1]) * stride;
                float x2 = (grid_x + regdfl[2]) * stride;
                float y2 = (grid_y + regdfl[3]) * stride;

                // 缩放到原图尺寸
                float xmin = x1 * scale_w;
                float ymin = y1 * scale_h;
                float xmax = x2 * scale_w;
                float ymax = y2 * scale_h;

                // 裁剪到图像边界
                xmin = std::max(0.0f, xmin);
                ymin = std::max(0.0f, ymin);
                xmax = std::min(static_cast<float>(orig_w), xmax);
                ymax = std::min(static_cast<float>(orig_h), ymax);

                detections.emplace_back(max_class_id, max_conf,
                                      xmin, ymin, xmax, ymax);
            }
        }
    }

    // 应用NMS
    return apply_nms(detections, config_.nms_threshold);
}

std::vector<DetectBox> YoloDetector::apply_nms(const std::vector<DetectBox>& detections,
                                               float iou_threshold) {
    if (detections.empty()) {
        return {};
    }

    // 创建索引数组并按置信度排序
    std::vector<int> indices(detections.size());
    for (size_t i = 0; i < detections.size(); ++i) {
        indices[i] = i;
    }

    std::sort(indices.begin(), indices.end(),
              [&detections](int a, int b) {
                  return detections[a].score > detections[b].score;
              });

    std::vector<bool> suppressed(detections.size(), false);
    std::vector<DetectBox> result;

    for (size_t i = 0; i < indices.size(); ++i) {
        int idx = indices[i];
        if (suppressed[idx]) {
            continue;
        }

        result.push_back(detections[idx]);
        cv::Rect rect_a = detections[idx].to_cv_rect();

        // 抑制重叠框
        for (size_t j = i + 1; j < indices.size(); ++j) {
            int idx2 = indices[j];
            if (suppressed[idx2]) {
                continue;
            }

            cv::Rect rect_b = detections[idx2].to_cv_rect();
            if (calculate_iou(rect_a, rect_b) > iou_threshold) {
                suppressed[idx2] = true;
            }
        }
    }

    return result;
}

float YoloDetector::calculate_iou(const cv::Rect& a, const cv::Rect& b) const {
    // 计算交集区域
    int x1 = std::max(a.x, b.x);
    int y1 = std::max(a.y, b.y);
    int x2 = std::min(a.x + a.width, b.x + b.width);
    int y2 = std::min(a.y + a.height, b.y + b.height);

    int w = std::max(0, x2 - x1);
    int h = std::max(0, y2 - y1);
    int intersection = w * h;

    int area_a = a.width * a.height;
    int area_b = b.width * b.height;

    return static_cast<float>(intersection) /
           static_cast<float>(area_a + area_b - intersection + 1e-6f);
}

} // namespace yolov11n_rknn
