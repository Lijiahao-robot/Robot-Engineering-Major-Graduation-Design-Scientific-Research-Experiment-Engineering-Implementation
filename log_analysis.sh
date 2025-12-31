#!/bin/bash
LOG_DIR="./robot_logs"
ANALYSIS_REPORT="./log_analysis_report.txt"

echo "===== 机器人日志分析报告 =====" > $ANALYSIS_REPORT
echo "分析时间：$(date '+%Y-%m-%d %H:%M:%S')" >> $ANALYSIS_REPORT
echo "日志目录：$LOG_DIR" >> $ANALYSIS_REPORT
echo "" >> $ANALYSIS_REPORT

# 1. 统计节点崩溃次数
echo "1. 节点崩溃统计：" >> $ANALYSIS_REPORT
grep -r "node down" $LOG_DIR | awk -F'[][]' '{print $4}' | sort | uniq -c >> $ANALYSIS_REPORT
echo "" >> $ANALYSIS_REPORT

# 2. 统计故障触发次数
echo "2. 故障触发统计：" >> $ANALYSIS_REPORT
grep -r "\[ERROR\]" $LOG_DIR | awk -F'[][]' '{print $6}' | cut -d' ' -f1 | sort | uniq -c >> $ANALYSIS_REPORT
echo "" >> $ANALYSIS_REPORT

# 3. 统计避障触发次数
echo "3. 避障触发统计：" >> $ANALYSIS_REPORT
grep -r "avoid obstacle" $LOG_DIR | wc -l >> $ANALYSIS_REPORT
echo "" >> $ANALYSIS_REPORT

# 4. 提取最新错误日志
echo "4. 最新10条错误日志：" >> $ANALYSIS_REPORT
grep -r "\[ERROR\]" $LOG_DIR | tail -10 >> $ANALYSIS_REPORT

echo "日志分析完成，报告保存至：$ANALYSIS_REPORT"
