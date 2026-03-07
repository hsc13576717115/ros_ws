#!/usr/bin/env python3
"""
YOLOv11 ONNX to RKNN Conversion Script

功能说明:
  1. 将 ONNX 格式的 YOLOv11 模型转换为 RKNN 格式
  2. 使用数据集进行 INT8 量化校准
  3. 测试转换后的模型并可视化结果

使用方法:
  1. 准备 ONNX 模型文件
  2. 准备量化校准数据集 (dataset.txt)
  3. 运行: python3 onnx2rknn_zq.py

注意:
  - 此脚本应在包的源码目录中运行
  - 所有路径使用相对路径，基于包的源码目录
"""

import os
import sys

# 获取脚本所在目录作为包的根目录
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = SCRIPT_DIR

# 添加包路径到系统路径
sys.path.insert(0, PACKAGE_ROOT)

import numpy as np
import cv2
from rknn.api import RKNN
from math import exp
from ament_index_python.packages import get_package_share_directory

# =============================================================================
# 配置参数（使用相对路径）
# =============================================================================

# 尝试从包安装目录获取路径，如果失败则使用源码目录
try:
    PACKAGE_SHARE_DIR = get_package_share_directory('yolov11n_rknn')
    # 使用安装目录
    MODELS_DIR = os.path.join(PACKAGE_SHARE_DIR, 'models')
    DATA_DIR = os.path.join(PACKAGE_SHARE_DIR, 'data')
    print(f"Using installed package directory: {PACKAGE_SHARE_DIR}")
except Exception as e:
    # 使用源码目录
    MODELS_DIR = os.path.join(PACKAGE_ROOT, 'models')
    DATA_DIR = os.path.join(PACKAGE_ROOT, 'data')
    print(f"Using source directory: {PACKAGE_ROOT}")

# 确保目录存在
os.makedirs(MODELS_DIR, exist_ok=True)
os.makedirs(DATA_DIR, exist_ok=True)

# 模型文件路径
ONNX_MODEL = os.path.join(MODELS_DIR, 'light.onnx')
RKNN_MODEL = os.path.join(MODELS_DIR, 'light.rknn')
DATASET = os.path.join(MODELS_DIR, 'dataset.txt')
TEST_IMAGE = os.path.join(DATA_DIR, 'image_10.jpg')
OUTPUT_IMAGE = os.path.join(MODELS_DIR, 'test_result.jpg')

# 量化配置
QUANTIZE_ON = True  # 是否启用 INT8 量化

# 目标平台
# - 'rk3588': OrangePi 5, RK3588
# - 'rk3566': OrangePi 4, RK3566
# - 'rk3568': RK3568
TARGET_PLATFORM = 'rk3588'

# 模型配置
CLASSES = ['ball']  # 类别名称
class_num = len(CLASSES)

# YOLOv11 配置 (640x640 输入)
headNum = 3
strides = [8, 16, 32]
mapSize = [[80, 80], [40, 40], [20, 20]]  # 640x640 对应的特征图尺寸
nmsThresh = 0.5
objectThresh = 0.5

# 输入分辨率 (640x640)
input_imgH = 640
input_imgW = 640

meshgrid = []


# =============================================================================
# 数据结构定义
# =============================================================================

class DetectBox:
    """检测框数据结构"""
    def __init__(self, classId, score, xmin, ymin, xmax, ymax):
        self.classId = classId
        self.score = score
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax


# =============================================================================
# 工具函数
# =============================================================================

def GenerateMeshgrid():
    """生成网格坐标"""
    for index in range(headNum):
        for i in range(mapSize[index][0]):
            for j in range(mapSize[index][1]):
                meshgrid.append(j + 0.5)
                meshgrid.append(i + 0.5)


def IOU(xmin1, ymin1, xmax1, ymax1, xmin2, ymin2, xmax2, ymax2):
    """计算两个边界框的 IoU"""
    xmin = max(xmin1, xmin2)
    ymin = max(ymin1, ymin2)
    xmax = min(xmax1, xmax2)
    ymax = min(ymax1, ymax2)

    innerWidth = xmax - xmin
    innerHeight = ymax - ymin

    innerWidth = innerWidth if innerWidth > 0 else 0
    innerHeight = innerHeight if innerHeight > 0 else 0

    innerArea = innerWidth * innerHeight

    area1 = (xmax1 - xmin1) * (ymax1 - ymin1)
    area2 = (xmax2 - xmin2) * (ymax2 - ymin2)

    total = area1 + area2 - innerArea

    return innerArea / total


def NMS(detectResult):
    """非极大值抑制"""
    predBoxs = []
    sort_detectboxs = sorted(detectResult, key=lambda x: x.score, reverse=True)

    for i in range(len(sort_detectboxs)):
        xmin1 = sort_detectboxs[i].xmin
        ymin1 = sort_detectboxs[i].ymin
        xmax1 = sort_detectboxs[i].xmax
        ymax1 = sort_detectboxs[i].ymax
        classId = sort_detectboxs[i].classId

        if sort_detectboxs[i].classId != -1:
            predBoxs.append(sort_detectboxs[i])
            for j in range(i + 1, len(sort_detectboxs), 1):
                if classId == sort_detectboxs[j].classId:
                    xmin2 = sort_detectboxs[j].xmin
                    ymin2 = sort_detectboxs[j].ymin
                    xmax2 = sort_detectboxs[j].xmax
                    ymax2 = sort_detectboxs[j].ymax
                    iou = IOU(xmin1, ymin1, xmax1, ymax1, xmin2, ymin2, xmax2, ymax2)
                    if iou > nmsThresh:
                        sort_detectboxs[j].classId = -1
    return predBoxs


def sigmoid(x):
    """Sigmoid 激活函数"""
    return 1 / (1 + exp(-x))


def postprocess(out, img_h, img_w):
    """
    YOLOv11 后处理函数

    Args:
        out: 模型输出张量列表
        img_h: 原始图像高度
        img_w: 原始图像宽度

    Returns:
        检测框列表
    """
    print('postprocess ... ')

    detectResult = []
    output = []
    for i in range(len(out)):
        print(f"  Output[{i}] shape: {out[i].shape}")
        output.append(out[i].reshape((-1)))

    scale_h = img_h / input_imgH
    scale_w = img_w / input_imgW

    gridIndex = -2
    cls_index = 0
    cls_max = 0

    for index in range(headNum):
        cls = output[index * 2 + 1]
        reg = output[index * 2 + 0]

        for h in range(mapSize[index][0]):
            for w in range(mapSize[index][1]):
                gridIndex += 2

                # 获取类别得分
                if 1 == class_num:
                    cls_max = sigmoid(cls[0 * mapSize[index][0] * mapSize[index][1] + h * mapSize[index][1] + w])
                    cls_index = 0
                else:
                    # 多类别：取最大类别
                    for cl in range(class_num):
                        cls_val = cls[cl * mapSize[index][0] * mapSize[index][1] + h * mapSize[index][1] + w]
                        if 0 == cl:
                            cls_max = cls_val
                            cls_index = cl
                        else:
                            if cls_val > cls_max:
                                cls_max = cls_val
                                cls_index = cl
                    cls_max = sigmoid(cls_max)

                # 置信度过滤
                if cls_max > objectThresh:
                    # DFL 解码
                    regdfl = []
                    for lc in range(4):
                        sfsum = 0
                        locval = 0
                        for df in range(16):
                            temp = exp(reg[((lc * 16) + df) * mapSize[index][0] * mapSize[index][1] + h * mapSize[index][1] + w])
                            reg[((lc * 16) + df) * mapSize[index][0] * mapSize[index][1] + h * mapSize[index][1] + w] = temp
                            sfsum += temp

                        for df in range(16):
                            sfval = reg[((lc * 16) + df) * mapSize[index][0] * mapSize[index][1] + h * mapSize[index][1] + w] / sfsum
                            locval += sfval * df
                        regdfl.append(locval)

                    # 计算边界框坐标
                    x1 = (meshgrid[gridIndex + 0] - regdfl[0]) * strides[index]
                    y1 = (meshgrid[gridIndex + 1] - regdfl[1]) * strides[index]
                    x2 = (meshgrid[gridIndex + 0] + regdfl[2]) * strides[index]
                    y2 = (meshgrid[gridIndex + 1] + regdfl[3]) * strides[index]

                    # 缩放到原图尺寸
                    xmin = x1 * scale_w
                    ymin = y1 * scale_h
                    xmax = x2 * scale_w
                    ymax = y2 * scale_h

                    # 裁剪到图像边界
                    xmin = xmin if xmin > 0 else 0
                    ymin = ymin if ymin > 0 else 0
                    xmax = xmax if xmax < img_w else img_w
                    ymax = ymax if ymax < img_h else img_h

                    box = DetectBox(cls_index, cls_max, xmin, ymin, xmax, ymax)
                    detectResult.append(box)

    # NMS
    print(f'  Raw detections: {len(detectResult)}')
    predBox = NMS(detectResult)
    print(f'  After NMS: {len(predBox)}')

    return predBox


def export_rknn_inference(img):
    """
    转换 ONNX 到 RKNN 并运行推理测试

    Args:
        img: 预处理后的输入图像 (NHWC 格式, uint8)

    Returns:
        模型输出张量列表
    """
    # Create RKNN object
    rknn = RKNN(verbose=False)

    # 配置模型
    print('--> Config model')
    print(f'  Target platform: {TARGET_PLATFORM}')
    print(f'  Quantization: {"Enabled" if QUANTIZE_ON else "Disabled"}')
    print(f'  Input size: {input_imgW}x{input_imgH}')

    rknn.config(
        mean_values=[[0, 0, 0]],
        std_values=[[255, 255, 255]],
        quantized_algorithm='normal',
        quantized_method='channel',
        target_platform=TARGET_PLATFORM
    )
    print('  done')

    # 加载 ONNX 模型
    print('--> Loading ONNX model')
    print(f'  ONNX model: {ONNX_MODEL}')
    ret = rknn.load_onnx(
        model=ONNX_MODEL,
        outputs=['reg1', 'cls1', 'reg2', 'cls2', 'reg3', 'cls3']
    )
    if ret != 0:
        print('  Load model failed!')
        exit(ret)
    print('  done')

    # 构建 RKNN 模型
    print('--> Building RKNN model')
    if QUANTIZE_ON:
        print(f'  Using dataset: {DATASET}')
        print(f'  Quantization: INT8')
    else:
        print(f'  Quantization: FP16 (no dataset needed)')

    ret = rknn.build(
        do_quantization=QUANTIZE_ON,
        dataset=DATASET if QUANTIZE_ON else None,
        rknn_batch_size=1
    )
    if ret != 0:
        print('  Build model failed!')
        exit(ret)
    print('  done')

    # 导出 RKNN 模型
    print('--> Export RKNN model')
    print(f'  Output: {RKNN_MODEL}')
    ret = rknn.export_rknn(RKNN_MODEL)
    if ret != 0:
        print('  Export rknn model failed!')
        exit(ret)
    print('  done')

    # 初始化运行时环境
    print('--> Init runtime environment')
    ret = rknn.init_runtime()
    if ret != 0:
        print('  Init runtime environment failed!')
        exit(ret)
    print('  done')

    # 运行推理测试
    print('--> Running inference test')
    outputs = rknn.inference(inputs=[img])
    rknn.release()
    print('  done')

    return outputs


# =============================================================================
# 主函数
# =============================================================================

if __name__ == '__main__':
    print('='*70)
    print('YOLOv11 ONNX to RKNN Conversion Script')
    print('='*70)
    print(f'Package root: {PACKAGE_ROOT}')
    print(f'Models directory: {MODELS_DIR}')
    print(f'Data directory: {DATA_DIR}')
    print(f'Input resolution: {input_imgW}x{input_imgH}')
    print(f'Feature maps: {mapSize}')
    print(f'Strides: {strides}')
    print('='*70)
    print()

    # 生成网格
    print('Generating meshgrid...')
    GenerateMeshgrid()
    print('  done')
    print()

    # 读取测试图像
    print(f'Loading test image: {TEST_IMAGE}')
    if not os.path.exists(TEST_IMAGE):
        print(f'  WARNING: Test image not found at: {TEST_IMAGE}')
        print(f'  Creating a placeholder test image...')
        # 创建一个简单的测试图像
        test_img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.imwrite(TEST_IMAGE, test_img)
        print(f'  Created test image at: {TEST_IMAGE}')

    orig_img = cv2.imread(TEST_IMAGE)
    img_h, img_w = orig_img.shape[:2]
    print(f'  Original size: {img_w}x{img_h}')
    print()

    # 预处理图像
    print('Preprocessing image...')
    print(f'  Resizing to: {input_imgW}x{input_imgH}')
    origimg = cv2.resize(orig_img, (input_imgW, input_imgH), interpolation=cv2.INTER_LINEAR)
    origimg = cv2.cvtColor(origimg, cv2.COLOR_BGR2RGB)
    img = np.expand_dims(origimg, 0)
    print(f'  Input shape: {img.shape}')
    print('  done')
    print()

    # 转换并推理
    outputs = export_rknn_inference(img)

    # 后处理
    out = []
    for i in range(len(outputs)):
        out.append(outputs[i])

    predbox = postprocess(out, img_h, img_w)

    print()
    print(f'Detection results: {len(predbox)} objects')

    # 绘制检测结果
    for i in range(len(predbox)):
        xmin = int(predbox[i].xmin)
        ymin = int(predbox[i].ymin)
        xmax = int(predbox[i].xmax)
        ymax = int(predbox[i].ymax)
        classId = predbox[i].classId
        score = predbox[i].score

        cv2.rectangle(orig_img, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
        ptext = (xmin, ymin)
        title = CLASSES[classId] + ":%.2f" % (score)
        cv2.putText(orig_img, title, ptext, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

    # 保存结果
    print()
    print(f'Saving result to: {OUTPUT_IMAGE}')
    cv2.imwrite(OUTPUT_IMAGE, orig_img)
    print('  done')
    print()
    print('='*70)
    print('Conversion completed successfully!')
    print('='*70)
