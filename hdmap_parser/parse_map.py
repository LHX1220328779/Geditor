#!/usr/bin/env python3
import sys
import os

# 确保能导入 common_msgs
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from common_msgs.map_msgs import map_pb2
except ImportError as e:
    print(f"❌ 导入错误: {e}")
    print("请确保 'protoc' 命令已成功执行。")
    sys.exit(1)

def main():
    if len(sys.argv) != 2:
        print("用法: python parse_map.py <map.bin>")
        sys.exit(1)

    map_path = sys.argv[1]
    if not os.path.exists(map_path):
        print(f"❌ 错误: 文件 {map_path} 不存在")
        sys.exit(1)

    # 读取并解析地图
    try:
        with open(map_path, 'rb') as f:
            hdmap = map_pb2.Map()
            hdmap.ParseFromString(f.read())
    except Exception as e:
        print(f"❌ 解析失败: {e}")
        sys.exit(1)

    # 打印统计信息
    print("✅ 成功解析高精地图!")
    print(f"- 车道数 (lanes):      {len(hdmap.lane)}")
    print(f"- 道路数 (roads):      {len(hdmap.road)}")
    print(f"- 路口数 (junctions):  {len(hdmap.junction)}")
    print(f"- 信号灯 (signals):    {len(hdmap.signal)}")

    # 打印前5条车道的详细信息
    print("\n🔍 车道详细信息（前5条）:")
    for i, lane in enumerate(hdmap.lane[:5]):
        print(f"\n--- 车道 {i+1} ---")
        print(f"ID: {lane.id.id}")
        print(f"类型: {lane.type}")
        print(f"长度: {lane.length:.2f} 米")
        print(f"限速: {lane.speed_limit:.2f} m/s")

        # 中心线点
        points = []
        if lane.central_curve.segment:
            seg = lane.central_curve.segment[0]
            if seg.HasField('line_segment'):
                points = seg.line_segment.point

        print(f"中心线点数: {len(points)}")
        for j, pt in enumerate(points[:3]):  # 只显示前3个点
            print(f"  点{j}: ({pt.x:.3f}, {pt.y:.3f}, {pt.z:.3f})")
        if len(points) > 3:
            print(f"  ... 还有 {len(points) - 3} 个点")

        # 正确获取前驱/后继车道 ID
        pred_ids = [pred.id for pred in lane.predecessor_id if pred.id.strip()]
        succ_ids = [succ.id for succ in lane.successor_id if succ.id.strip()]

        print(f"前驱车道: {pred_ids}")
        print(f"后继车道: {succ_ids}")

if __name__ == "__main__":
    main()