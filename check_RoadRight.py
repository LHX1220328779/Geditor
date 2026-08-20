"""
# 功能：
    检查自行设定的均匀碾压路段.txt中的路段是否与均匀碾压路段.xlsx中所开均匀碾压的路段一致
# 使用方法：
    1. 确保路段.txt和均匀碾压路段.xlsx文件均放在 */场景文件 目录下
    2. 运行以下命令：
    python check_RoadRight.py --dir */场景文件
"""

import pandas as pd
import os
import argparse


def find_files_recursive(base_dir):
    lu_duan_files = []
    xlsx_files = []
    for root, dirs, files in os.walk(base_dir):
        for f in files:
            full_path = os.path.join(root, f)
            if f.endswith("路段.txt"):
                lu_duan_files.append(full_path)
            elif f.endswith(".xlsx") and not f.startswith("~$"):
                xlsx_files.append(full_path)
    return lu_duan_files, xlsx_files


def parse_txt_roads(txt_path):
    txt_roads = set()
    try:
        with open(txt_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()

        start_collecting = False
        for line in lines:
            line = line.strip()
            if not line:
                continue
            if "均匀碾压路段：" in line:
                start_collecting = True
                continue
            if "修改路段：" in line:
                start_collecting = False
                continue

            if start_collecting:
                if line != "无":
                    txt_roads.add(line)
    except Exception as e:
        print(f"读取 TXT 文件失败 {txt_path}: {e}")
    return txt_roads


def parse_xlsx_roads(xlsx_path):
    excel_roads = set()
    all_road_codes = set()
    try:
        df = pd.read_excel(xlsx_path)
        if '均匀碾压' not in df.columns or '路段编码' not in df.columns:
            return excel_roads, all_road_codes, False
        all_road_codes = set(df['路段编码'].astype(str).str.strip().unique())
        enabled_mask = df['均匀碾压'].astype(str).str.strip() == '1'
        excel_roads = set(df.loc[enabled_mask, '路段编码'].astype(str).str.strip().unique())
    except Exception as e:
        print(f"读取 Excel 文件失败 {xlsx_path}: {e}")
        return excel_roads, all_road_codes, False
    return excel_roads, all_road_codes, True


def print_report(xlsx_name, matched_roads, only_in_txt, only_in_excel):
    print("=" * 50)
    print(f"核对报告: {xlsx_name}")
    print("=" * 50)

    print(f"\n[1] 成功匹配的路段 (共 {len(matched_roads)} 个):")
    if matched_roads:
        for road in sorted(list(matched_roads)):
            print(f"  - {road}")
    else:
        print("  无")

    print(f"\n[2] 不一致情况 A：TXT 中存在，但 Excel 中未设置为'1' (共 {len(only_in_txt)} 个):")
    if only_in_txt:
        for road in sorted(list(only_in_txt)):
            print(f"  ❌ 错误: {road}")
    else:
        print("  ✅ 无不一致")

    print(f"\n[3] 不一致情况 B：Excel 中已设置为'1'，但 TXT 中不存在 (共 {len(only_in_excel)} 个):")
    if only_in_excel:
        for road in sorted(list(only_in_excel)):
            print(f"  ❌ 错误: {road}")
    else:
        print("  ✅ 无不一致")

    print()


def main():
    parser = argparse.ArgumentParser(description="核对路段信息")
    parser.add_argument(
        "--dir",
        type=str,
        default="/home/project/HDMap_data/shuangfeng/路线包/test",
        help="包含路段.txt和xlsx文件的根目录路径",
    )
    args = parser.parse_args()

    base_dir = args.dir
    if not os.path.isdir(base_dir):
        print(f"[错误] 目录不存在: {base_dir}")
        return

    print(f"扫描目录: {base_dir}")
    lu_duan_files, xlsx_files = find_files_recursive(base_dir)

    print(f"找到 {len(lu_duan_files)} 个路段文件:")
    for f in lu_duan_files:
        print(f"  - {f}")
    print(f"找到 {len(xlsx_files)} 个xlsx文件:")
    for f in xlsx_files:
        print(f"  - {f}")

    if not lu_duan_files:
        print("[错误] 未找到任何路段.txt文件")
        return
    if not xlsx_files:
        print("[错误] 未找到任何xlsx文件")
        return

    all_txt_roads = set()
    for txt_file in lu_duan_files:
        roads = parse_txt_roads(txt_file)
        all_txt_roads = all_txt_roads.union(roads)

    print(f"\n路段.txt中共有 {len(all_txt_roads)} 个均匀碾压路段")

    xlsx_data_map = {}
    all_excel_roads_union = set()
    for xlsx_file in xlsx_files:
        excel_roads, all_road_codes, has_columns = parse_xlsx_roads(xlsx_file)
        xlsx_name = os.path.join(os.path.basename(os.path.dirname(xlsx_file)), os.path.basename(xlsx_file))
        if not has_columns:
            print(f"\n跳过 {xlsx_name} (缺少'均匀碾压'或'路段编码'列)")
            continue
        xlsx_data_map[xlsx_file] = (excel_roads, all_road_codes)
        all_excel_roads_union = all_excel_roads_union.union(excel_roads)

    has_any_error = False

    for xlsx_file in xlsx_data_map:
        xlsx_name = os.path.join(os.path.basename(os.path.dirname(xlsx_file)), os.path.basename(xlsx_file))
        excel_roads, all_road_codes = xlsx_data_map[xlsx_file]

        matched_roads = all_txt_roads.intersection(excel_roads)
        only_in_txt = all_txt_roads.intersection(all_road_codes) - excel_roads
        only_in_excel = excel_roads - all_txt_roads

        if only_in_txt or only_in_excel:
            has_any_error = True

        print_report(xlsx_name, matched_roads, only_in_txt, only_in_excel)

    print("=" * 50)
    if not has_any_error:
        print("最终结果：数据完全一致！")
    else:
        print("最终结果：数据存在差异，请检查上述错误列表。")
    print("=" * 50)


if __name__ == "__main__":
    main()
