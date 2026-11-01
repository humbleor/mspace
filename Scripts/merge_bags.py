#!/usr/bin/env python3
"""合并 ROS1 分包，保留原文件。先加载 ROS 环境。

用法：python3 Scripts/merge_bags.py -o ~/data/merged.bag ~/data/flight_*.bag
只选择同一次录制的分包；录制结束、所有 .active 文件关闭后再运行。
"""

import argparse
import glob
import heapq
from contextlib import ExitStack
from pathlib import Path

import rosbag


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "-o", "--output", required=True, help="输出 bag 路径（不能已存在）"
    )
    parser.add_argument("inputs", nargs="+", help="分包路径，支持通配符")
    args = parser.parse_args()
    output = Path(args.output).expanduser().resolve()
    if output.exists():
        parser.error("输出文件已存在，请换一个名称：{}".format(output))

    inputs = []
    for pattern in args.inputs:
        matches = glob.glob(str(Path(pattern).expanduser()))
        if not matches:
            parser.error("没有匹配的分包：{}".format(pattern))
        inputs.extend(Path(path).resolve() for path in matches)
    inputs = list(dict.fromkeys(inputs))
    for path in inputs:
        if not path.is_file() or path.suffix != ".bag":
            parser.error("输入必须是已结束录制的 .bag 文件：{}".format(path))
        if path == output:
            parser.error("输出不能同时作为输入")

    count = 0
    with ExitStack() as stack:
        bags = [stack.enter_context(rosbag.Bag(str(path), "r")) for path in inputs]
        # 原始消息无需反序列化，保留自定义消息定义和锁存信息。
        streams = [
            bag.read_messages(raw=True, return_connection_header=True) for bag in bags
        ]
        with output.open("xb") as file:
            with rosbag.Bag(file, "w", compression="lz4") as merged:
                for topic, msg, timestamp, header in heapq.merge(
                    *streams, key=lambda record: record.timestamp.to_nsec()
                ):
                    merged.write(
                        topic, msg, timestamp, raw=True, connection_header=header
                    )
                    count += 1
    print("已合并 {} 个分包，共 {} 条消息：{}".format(len(inputs), count, output))


if __name__ == "__main__":
    main()
