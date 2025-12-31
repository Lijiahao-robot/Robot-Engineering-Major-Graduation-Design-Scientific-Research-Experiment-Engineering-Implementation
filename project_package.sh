#!/bin/bash
PROJECT_NAME="机器人自主导航系统_毕设项目"
PACKAGE_DIR="./$PROJECT_NAME"
ZIP_NAME="$PROJECT_NAME_2027毕设提交版.zip"

# 1. 创建打包目录
rm -rf $PACKAGE_DIR $ZIP_NAME
mkdir -p $PACKAGE_DIR/{代码包,文档包,视频包,报告包}

# 2. 复制代码包（排除编译文件、日志文件，只保留源码+配置）
cp -r ./src ./config ./scripts ./utils $PACKAGE_DIR/代码包
rm -rf $PACKAGE_DIR/代码包/**/build $PACKAGE_DIR/代码包/**/install $PACKAGE_DIR/代码包/**/log
echo "代码包复制完成"

# 3. 复制文档包（开题报告+毕业论文+专利素材+PPT）
cp -r ./文档/* $PACKAGE_DIR/文档包
echo "文档包复制完成"

# 4. 复制视频包（成果展示视频+仿真演示视频+应急录屏）
cp -r ./视频/* $PACKAGE_DIR/视频包
echo "视频包复制完成"

# 5. 复制报告包（性能评估报告+日志分析报告+硬件检测报告）
cp ./performance_report.csv ./log_analysis_report.txt ./hardware_check.log $PACKAGE_DIR/报告包
echo "报告包复制完成"

# 6. 生成提交说明文档
echo "## 机器人自主导航系统 毕设项目 提交说明
### 一、 目录结构
1.  代码包：所有ROS2功能包、脚本、工具类代码，可直接编译运行
2.  文档包：开题报告、毕业论文（定稿版）、答辩PPT、专利申报素材
3.  视频包：成果展示视频（3分钟）、仿真演示视频、应急录屏
4.  报告包：性能评估报告、日志分析报告、硬件检测报告

### 二、 编译运行步骤
1.  进入代码包，执行 colcon build
2.  source install/setup.bash
3.  执行 ./scripts/all_in_one_launch.sh 启动所有节点

### 三、 备注
- 若真实硬件无法启动，运行 ./scripts/data_playback_node.py 回放离线数据演示
- 答辩演示顺序：硬件全貌→仿真建图→避障演示→语音/APP控制→性能报告" > $PACKAGE_DIR/提交说明.md

# 7. 压缩打包
zip -r $ZIP_NAME $PACKAGE_DIR
echo "项目打包完成！最终提交文件：$ZIP_NAME"
echo "打包目录结构："
tree $PACKAGE_DIR
