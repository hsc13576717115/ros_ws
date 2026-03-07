#!/usr/bin/env python3
"""
自动生成量化校准数据集列表

功能说明:
  - 扫描 data/ 目录中的所有图像文件
  - 自动生成 models/dataset.txt 文件
  - 支持常见图像格式: jpg, png, jpeg, bmp

使用方法:
  python3 update_dataset.py

可选参数:
  --data-dir    数据目录 (默认: data/)
  --output      输出文件 (默认: models/dataset.txt)
  --relative    使用相对路径 (默认: 绝对路径)
  --shuffle     随机打乱顺序
  --limit N     限制图片数量
"""

import os
import sys
import argparse
from pathlib import Path

# 获取脚本所在目录
SCRIPT_DIR = Path(__file__).parent.absolute()
PACKAGE_ROOT = SCRIPT_DIR

# 默认配置
DEFAULT_DATA_DIR = PACKAGE_ROOT / "data"
DEFAULT_OUTPUT = PACKAGE_ROOT / "models" / "dataset.txt"

# 支持的图像格式
IMAGE_EXTENSIONS = {
    '.jpg', '.jpeg', '.png', '.bmp',
    '.JPG', '.JPEG', '.PNG', '.BMP'
}


def find_images(data_dir, recursive=True):
    """
    扫描目录中的所有图像文件

    Args:
        data_dir: 数据目录路径
        recursive: 是否递归搜索子目录

    Returns:
        图像文件路径列表
    """
    image_files = []

    if not data_dir.exists():
        print(f"❌ 错误: 数据目录不存在: {data_dir}")
        return image_files

    if recursive:
        # 递归搜索
        for ext in IMAGE_EXTENSIONS:
            image_files.extend(data_dir.rglob(f"*{ext}"))
    else:
        # 仅搜索当前目录
        for ext in IMAGE_EXTENSIONS:
            image_files.extend(data_dir.glob(f"*{ext}"))

    # 排序（保证顺序稳定）
    image_files.sort()

    return image_files


def generate_dataset_txt(image_files, output_file, data_dir, relative_to=None, shuffle=False, limit=None):
    """
    生成 dataset.txt 文件

    Args:
        image_files: 图像文件路径列表
        output_file: 输出文件路径
        data_dir: 数据目录路径（用于说明文件）
        relative_to: 转换为相对路径的基准目录
        shuffle: 是否打乱顺序
        limit: 限制文件数量
    """
    # 应用限制
    if limit and limit < len(image_files):
        import random
        print(f"⚠️  限制图片数量: {limit}/{len(image_files)}")
        image_files = image_files[:limit]

    # 打乱顺序
    if shuffle:
        import random
        random.shuffle(image_files)
        print("🔀 已打乱图片顺序")

    # 转换路径
    output_lines = []
    for img_path in image_files:
        if relative_to:
            # 转换为相对路径
            try:
                rel_path = img_path.relative_to(relative_to)
                output_lines.append(str(rel_path))
            except ValueError:
                # 如果无法转换为相对路径，使用绝对路径
                output_lines.append(str(img_path.absolute()))
        else:
            # 使用绝对路径
            output_lines.append(str(img_path.absolute()))

    # 写入文件
    output_file.parent.mkdir(parents=True, exist_ok=True)

    # 写入 dataset.txt (纯路径，RKNN Toolkit 不支持注释)
    with open(output_file, 'w', encoding='utf-8') as f:
        for line in output_lines:
            f.write(line + '\n')

    # 生成说明文件
    info_file = output_file.with_suffix('.txt.info')
    with open(info_file, 'w', encoding='utf-8') as f:
        f.write("# 量化校准数据集说明\n")
        f.write(f"# 生成时间: {__import__('datetime').datetime.now()}\n")
        f.write(f"# 数据目录: {data_dir}\n")
        f.write(f"# 图片数量: {len(output_lines)}\n")
        f.write("#\n")
        f.write("# 使用说明:\n")
        f.write("#   - 当前使用相对路径 (相对于包根目录)\n")
        f.write("#   - RKNN Toolkit 不支持注释，所以说明单独存放在此文件\n")
        f.write("#   - 添加更多图片以提高量化精度 (推荐 50-200 张)\n")
        f.write("#\n")
        f.write("# 重新生成数据集:\n")
        f.write("#   python3 update_dataset.py\n")
        f.write("#\n")
        if relative_to:
            f.write(f"# 路径基准: {relative_to}\n")
        f.write("#\n")
        f.write("# 前 10 个文件:\n")
        for i, line in enumerate(output_lines[:10], 1):
            f.write(f"#   {i}. {line}\n")
        if len(output_lines) > 10:
            f.write(f"#   ... 还有 {len(output_lines) - 10} 个文件\n")

    print(f"✅ 已生成: {output_file} (纯路径)")
    print(f"✅ 已生成: {info_file} (说明文档)")
    print(f"   图片数量: {len(output_lines)}")


def main():
    parser = argparse.ArgumentParser(
        description='自动生成量化校准数据集列表',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
示例:
  # 使用默认配置 (绝对路径，推荐用于 RKNN Toolkit)
  python3 update_dataset.py

  # 使用相对路径 (用于跨工作空间)
  python3 update_dataset.py --relative

  # 递归搜索所有子目录
  python3 update_dataset.py --recursive

  # 随机打乱并限制数量
  python3 update_dataset.py --shuffle --limit 50

  # 指定自定义目录
  python3 update_dataset.py --data-dir /path/to/images --output my_dataset.txt
        '''
    )

    parser.add_argument(
        '--data-dir',
        type=str,
        default=str(DEFAULT_DATA_DIR),
        help=f'数据目录 (默认: {DEFAULT_DATA_DIR})'
    )

    parser.add_argument(
        '--output',
        type=str,
        default=str(DEFAULT_OUTPUT),
        help=f'输出文件 (默认: {DEFAULT_OUTPUT})'
    )

    parser.add_argument(
        '--relative',
        action='store_true',
        help='使用相对路径 (默认: 绝对路径)'
    )

    parser.add_argument(
        '--no-recursive',
        action='store_true',
        help='不递归搜索子目录'
    )

    parser.add_argument(
        '--shuffle',
        action='store_true',
        help='随机打乱图片顺序'
    )

    parser.add_argument(
        '--limit',
        type=int,
        metavar='N',
        help='限制图片数量'
    )

    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='仅显示找到的图片，不写入文件'
    )

    args = parser.parse_args()

    # 转换路径
    data_dir = Path(args.data_dir)
    output_file = Path(args.output)

    # 查找图片
    print(f"🔍 扫描目录: {data_dir}")
    recursive = not args.no_recursive
    image_files = find_images(data_dir, recursive=recursive)

    if not image_files:
        print("❌ 未找到任何图像文件")
        print(f"   支持的格式: {', '.join(IMAGE_EXTENSIONS)}")
        sys.exit(1)

    # 显示找到的图片
    print(f"✅ 找到 {len(image_files)} 个图像文件:")
    for i, img in enumerate(image_files[:10], 1):
        print(f"   {i:2d}. {img.name}")
    if len(image_files) > 10:
        print(f"   ... 还有 {len(image_files) - 10} 个文件")

    # Dry run 模式
    if args.dry_run:
        print("\n📋 预览 (前5行):")
        for img in image_files[:5]:
            if not args.relative:
                print(f"   {img.absolute()}")
            else:
                print(f"   {img.relative_to(PACKAGE_ROOT)}")
        return

    # 生成文件
    relative_base = None if not args.relative else PACKAGE_ROOT
    generate_dataset_txt(
        image_files,
        output_file,
        data_dir,
        relative_to=relative_base,
        shuffle=args.shuffle,
        limit=args.limit
    )

    # 显示建议
    print(f"\n💡 使用建议:")
    if len(image_files) < 20:
        print(f"   ⚠️  当前图片数量较少 ({len(image_files)} 张)")
        print(f"   建议: 添加更多图片 (50-200 张) 以获得更好的量化效果")
    else:
        print(f"   ✅ 图片数量充足 ({len(image_files)} 张)")

    print(f"\n📖 查看文件:")
    print(f"   cat {output_file}")


if __name__ == '__main__':
    main()
