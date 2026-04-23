#!/bin/bash

# 检查参数数量
if [ $# -ne 4 ]; then
    echo "Usage: $0 <type> <input_path> <output_path> <version>"
    echo "type: 0 (both), 1 (vdb2pb), 2 (vdb2dynamic)"
    exit 1
fi

# 获取参数
TYPE=$1
INPUT_PATH=$2
OUTPUT_PATH=$3
VERSION=$4

# 检查类型参数是否有效
if [[ ! $TYPE =~ ^[0-2]$ ]]; then
    echo "Error: Invalid type. Must be 0, 1, or 2"
    exit 1
fi

# 执行vdb2pb程序
run_vdb2pb() {
    echo "Running vdb2pb..."
    ./bin/vdb2pb "$INPUT_PATH" "$OUTPUT_PATH" "$VERSION"
}

# 执行vdb2dynamic程序
run_vdb2dynamic() {
    echo "Running vdb2dynamic..."
    ./bin/vdb2dynamic "$INPUT_PATH" "$OUTPUT_PATH" "$VERSION"
}

# 根据类型执行相应程序
case $TYPE in
    0)
        run_vdb2pb
        run_vdb2dynamic
        ;;
    1)
        run_vdb2pb
        ;;
    2)
        run_vdb2dynamic
        ;;
esac