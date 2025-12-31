#!/bin/bash
PACKAGE_DIR="./机器人自主导航系统_毕设项目"

echo "===== 毕设项目提交校验 ====="
CHECK_RESULT="通过"

# 校验核心文件
REQUIRED_FILES=(
    "$PACKAGE_DIR/代码包/src"
    "$PACKAGE_DIR/文档包/毕业论文_定稿版.docx"
    "$PACKAGE_DIR/文档包/答辩PPT.pptx"
    "$PACKAGE_DIR/视频包/成果展示视频.mp4"
    "$PACKAGE_DIR/报告包/performance_report.csv"
    "$PACKAGE_DIR/提交说明.md"
)

for file in "${REQUIRED_FILES[@]}"; do
    if [ ! -f "$file" ] && [ ! -d "$file" ]; then
        echo "❌ 缺失关键文件：$file"
        CHECK_RESULT="失败"
    else
        echo "✅ $file 存在"
    fi
done

echo ""
if [ "$CHECK_RESULT" == "通过" ]; then
    echo "✅ 校验通过！可正常提交毕设项目"
else
    echo "❌ 校验失败！请补充缺失的关键文件"
fi
