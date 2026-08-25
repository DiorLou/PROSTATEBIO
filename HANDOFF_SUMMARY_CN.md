# 前列腺 TRUS 活检针识别与机器人反馈控制项目交接摘要

最后更新：2026-08-25
当前工作区：`C:\Users\Administrator\Desktop\PROSTATEBIO`
当前分支：`main`

> 用途：换电脑或开启新 Codex 对话后，先读取本文件，即可接着当前研究和代码状态继续工作。不要依赖旧电脑上的虚拟环境；重新创建环境并安装依赖。

---

## 1. 当前研究目标

项目目标是实现体模前列腺 TRUS 图像中的活检针识别，并把识别结果用于机器人活检反馈控制。

现在研究逻辑已经从“单一针识别模型”拆成两个模型：

1. **模型1：超声中心选择模型**
   - 控制 yaw 做固定小范围搜索；
   - 输入 7 张 TRUS 图像；
   - 输出应该转到哪一个 yaw 姿态；
   - 目标是让针位于超声束中心附近，保证后续图像可用于反馈。

2. **模型2：针几何识别模型**
   - 只在模型1选出的可用图像上运行；
   - 输入 `IN_CENTER` 图像和先验 mask；
   - 输出未激发针尖 `tip` 和针轴方向 `shaft`；
   - 由未激发针尖沿针轴前方约 10 mm 推算激发后的活检中心；
   - 再用于 pitch 反馈控制。

整体控制思路：

```text
固定 yaw 搜索采集 7 帧
→ 模型1选择位于超声中心的 yaw 姿态
→ yaw 校正
→ 模型2识别 tip / shaft
→ 推算激发后活检中心
→ pitch 反馈调整针方向
→ 继续进针
→ 激发活检枪
```

当前论文实验只使用**体模数据**，不要写成临床数据。

---

## 2. 最近 Git 提交

当前工作区在提交前已确认干净。最近关键提交：

```text
bf44e06 Harden model1 training pipeline
a0a7921 Add model1 yaw sweep capture
be5cb94 Update Chinese handoff summary
6b12a53 Add yaw center selector model
2ba6dda Add TCP_U in volume
858dcf8 Add phantom prior mask labeling tools
570a738 Target biopsy center in pitch feedback
ced8360 Use unfired needle tip for ultrasound prior box
f91fae5 Add mini-book submodule
f046c3b Fix ultrasound kinematic box projection axes
```

重要历史改动：

- `f046c3b`：修正超声图像投影轴关系；
- `ced8360`：保存和 prior box 使用未激发针尖，J3 减去 26 mm；
- `570a738`：pitch feedback 改为控制激发后活检中心；
- `858dcf8`：添加体模 manifest、prior mask、overlay 预览和人工 `IN_PLANE/OUT_OF_PLANE` 标注工具；
- `6b12a53`：添加模型1的 7 帧 yaw 中心选择训练与预测代码。
- `a0a7921`：添加模型1的针 yaw 逐度采集、Beckhoff Ready 同步保存、prior mask 和逐帧 CSV。
- `bf44e06`：修复并强化模型1训练/预测流水线，添加 CSV 汇总、类别评估和自动测试。

---

## 3. Python 环境

仓库中存在：

```text
.venv311/
requirements.txt
mini-book/requirements.txt
```

不要直接复制旧虚拟环境到新电脑继续用。建议使用 Python 3.11 重新创建环境。

主项目依赖中包含：

```text
torch
torchvision
opencv-python
pillow
numpy
pandas
PyQt5
```

当前默认 `python` 环境仍会出现 `ModuleNotFoundError: No module named 'torch'`。仓库中的 `.venv311` 已确认包含 CPU 版 `torch 2.9.1` 和 `torchvision 0.24.1`，模型1命令应使用：

```powershell
.\.venv311\Scripts\python.exe
```

当前 `torch.cuda.is_available()` 为 `False`，可以训练，但正式大数据训练会比 GPU 慢。

---

## 4. 数据和生成文件

### 4.1 原始体模图像

原始图片目录：

```text
image/
```

当前图像主要来自实时采集文件夹，例如：

```text
image/Realtime_Capture_Interval_300ms_YYYYMMDD_HHMMSS/
```

实时保存图像文件名中包含：

```text
E(...)
TipU(...)
VecU(...)
Box(...)
```

### 4.2 Manifest 和 prior masks

已生成：

```text
phantom_manifest.csv
phantom_prior_masks/
phantom_in_plane_labels.csv
phantom_prior_mask_check_tip_extend_6mm/
```

`phantom_manifest.csv` 中记录：

- 图像路径；
- sequence/frame；
- 原始文件名中的 fired tip；
- 重新计算的 unfired tip；
- needle vector；
- biopsy center；
- prior mask 路径；
- prior box 四角；
- 投影点是否在图像内。

`prior mask` 中文建议写作：

```text
先验掩膜 / 运动学先验掩膜
```

含义：模型看图像之前，根据机器人运动学预先知道“针理论上可能出现的区域”。它不是人工真值。

### 4.3 prior mask 当前参数

关键脚本：

```text
build_phantom_manifest.py
```

关键常量：

```python
MM_PER_PIXEL = 66.0 / 420.0
TCP_U_ORIGIN_U_PX = 406.0
TCP_U_ORIGIN_V_PX = 420.0 + 10.0 / MM_PER_PIXEL
UNFIRED_NEEDLE_RETRACTION_MM = 26.0
BIOPSY_CENTER_OFFSET_MM = 10.0
KINEMATIC_BOX_HALF_WIDTH_PX = 25.0
KINEMATIC_BOX_TIP_EXTENSION_MM = 6.0
```

当前确认使用 **6 mm tip-side extension**。之前试过 16 mm，太长；6 mm 用户确认可以。

最新检查图：

```text
phantom_prior_mask_check_tip_extend_6mm/prior_mask_check_tip_extend_6mm_montage.png
```

---

## 5. 超声投影关系

当前已确认图像投影关系：

```text
u = u0 - TCP_U.z / mm_per_pixel
v = v0 + TCP_U.y / mm_per_pixel
```

二维向量投影：

```text
du = -VecU.z / mm_per_pixel
dv = +VecU.y / mm_per_pixel
```

重要背景：

- 旧关系曾使用 `v = v0 - TipU.x/mm`，导致投影点大多不在图像内；
- 后来通过调试叠加图确认替代投影关系正确；
- prior box 半宽从 ±10 px 改到 ±18 px，再改到 ±25 px；
- 当前使用 ±25 px。

注意：用户后来讨论探头几何时指出，实际 yaw 参考向量 `(0, 1, 0)` 属于 `TCP_P`，不是 `TCP_U`。不要在论文示意图中乱画 TCP_U 坐标系方向。

---

## 6. 未激发针尖与活检中心

关键事实：

- 识别出来的实际针尖是**未激发状态下的针尖**；
- 机器人运动学中 `robot.get_tip_of_needle([...])` 原本对应激发后针尖；
- 未激发针比激发后短约 26 mm；
- 所以计算未激发针尖时使用：

```python
tip_joint_values = joint_values.copy()
tip_joint_values[3] -= 26.0
tip_p = robot.get_tip_of_needle(tip_joint_values)
```

控制目标不是让未激发针尖到病灶，而是让激发后活检中心到病灶。

当前定义：

```text
活检中心 = 未激发针尖 + 10 mm * 针方向
```

对应代码在：

```text
core/needle_pitch_feedback.py
```

并已有测试：

```text
tests/test_needle_pitch_feedback.py
```

测试曾通过：

```text
python -m unittest tests.test_needle_pitch_feedback -v
```

---

## 7. 人工标注流程

当前推荐流程：

```text
原始图片
→ 自动生成 manifest + prior_mask + overlay preview
→ 人工看 overlay preview，挑需要标的图
→ 单独 CSV 标 IN_PLANE / OUT_OF_PLANE
→ LabelMe 只对 IN_PLANE 图像标 tip / shaft
→ 汇总成训练集
```

原因：

- `NO_NEEDLE` 不需要模型专门识别，可由运动学投影是否在图像区域内初筛；
- 基本只要 `IN_PLANE`，就属于可用于反馈；
- 只有 `IN_PLANE / IN_CENTER` 图像才值得标注针尖和针轴。

已有人工浏览工具：

```text
review_in_plane_labels.py
```

运行：

```powershell
python review_in_plane_labels.py
```

按键：

```text
右箭头 / 空格：下一张
左箭头：上一张
i：标 IN_PLANE 并下一张
o：标 OUT_OF_PLANE 并下一张
u：清除当前标签
q：退出
```

实时写入：

```text
phantom_in_plane_labels.csv
```

overlay 颜色含义：

```text
红点：未激发针尖
蓝点：激发后活检中心
绿线：针轴中心线
黄色框/透明区域：prior mask
```

---

## 8. 模型1：超声中心选择模型

最新决定：模型1不是单帧二分类，而是**7 帧序列输入、8 类输出**。

### 8.1 采集动作

以当前 yaw 为 0 度：

```text
先 needle rotate left 3 deg
然后每次 right 1 deg
一共采 7 张图
```

对应 yaw offset：

```text
-3, -2, -1, 0, +1, +2, +3
```

### 8.2 人工标注

每张图先人工标注：

```text
IN_CENTER
OFF_CENTER
```

`IN_CENTER` 表示：

- 针处于超声束中心附近；
- 针影足够可见；
- 图像可进入模型2做 tip / shaft 识别。

`OFF_CENTER` 表示：

- 针偏离超声中心；
- 或针影不够稳定/不适合后续反馈。

### 8.3 序列标签生成规则

模型输出不是每张图的标签，而是在 7 张图中选择目标 yaw。

若有多个 `IN_CENTER`：

```text
选择离 0 度最近的一张
若正负距离相同，优先选择负方向
```

优先级：

```text
0 → -1 → +1 → -2 → +2 → -3 → +3 → NO_VALID
```

输出 8 类：

```text
YAW_M3
YAW_M2
YAW_M1
YAW_0
YAW_P1
YAW_P2
YAW_P3
NO_VALID
```

如果 7 张都不是 `IN_CENTER`，输出：

```text
NO_VALID
```

### 8.4 已实现代码

目录：

```text
model1_center_selector/
```

文件：

```text
model1_center_selector/README.md
model1_center_selector/label_rules.py
model1_center_selector/build_sequences.py
model1_center_selector/dataset.py
model1_center_selector/model.py
model1_center_selector/train.py
model1_center_selector/predict.py
```

模型结构：

```text
7 张 TRUS 图像
→ 共享 ResNet18 encoder
→ 拼接 7 帧特征
→ MLP
→ 8 类输出
```

支持两种输入：

```text
仅 TRUS 灰度图
TRUS 灰度图 + prior mask
```

### 8.5 逐帧标注 CSV

输入格式：

```csv
sequence_id,yaw_offset_deg,image_path,mask_path,center_label
seq_0001,-3,image/seq_0001/m3.png,mask/seq_0001/m3.png,OFF_CENTER
seq_0001,-2,image/seq_0001/m2.png,mask/seq_0001/m2.png,OFF_CENTER
seq_0001,-1,image/seq_0001/m1.png,mask/seq_0001/m1.png,IN_CENTER
seq_0001,0,image/seq_0001/0.png,mask/seq_0001/0.png,OFF_CENTER
seq_0001,1,image/seq_0001/p1.png,mask/seq_0001/p1.png,IN_CENTER
seq_0001,2,image/seq_0001/p2.png,mask/seq_0001/p2.png,OFF_CENTER
seq_0001,3,image/seq_0001/p3.png,mask/seq_0001/p3.png,OFF_CENTER
```

生成序列训练 CSV：

```powershell
.\.venv311\Scripts\python.exe -m model1_center_selector.merge_frame_labels `
  --capture-root image/model1_center_capture `
  --output-csv model1_center_frame_labels.csv

.\.venv311\Scripts\python.exe -m model1_center_selector.build_sequences `
  --frame-csv model1_center_frame_labels.csv `
  --output-csv model1_center_sequences.csv
```

上面例子中，`-1` 和 `+1` 都是 `IN_CENTER`，会自动选择 `-1`。

### 8.6 训练

不使用 prior mask：

```powershell
.\.venv311\Scripts\python.exe -m model1_center_selector.train `
  --train-csv model1_center_sequences.csv `
  --root-dir . `
  --output runs/model1_center_selector/best.pt
```

使用 prior mask：

```powershell
.\.venv311\Scripts\python.exe -m model1_center_selector.train `
  --train-csv model1_center_sequences.csv `
  --root-dir . `
  --use-masks `
  --class-balanced-loss `
  --output runs/model1_center_selector/best.pt
```

训练默认启用 ImageNet 预训练 ResNet18。正式实验推荐使用来自独立采集批次的 `--val-csv`，避免相似序列随机分到训练和验证两侧。训练会记录总体准确率、宏平均 F1、各类召回率和混淆矩阵，最佳模型按验证集宏平均 F1 保存。

训练输出目录：

```text
runs/
```

已加入 `.gitignore`。

### 8.7 预测

预测时也按 yaw 顺序输入 7 张图：

```text
-3, -2, -1, 0, +1, +2, +3
```

命令示例：

```powershell
.\.venv311\Scripts\python.exe -m model1_center_selector.predict `
  --checkpoint runs/model1_center_selector/best.pt `
  --images m3.png m2.png m1.png zero.png p1.png p2.png p3.png `
  --masks m3_mask.png m2_mask.png m1_mask.png zero_mask.png p1_mask.png p2_mask.png p3_mask.png `
  --output-json prediction.json
```

输出：

```text
pred_class
pred_name
pred_yaw_offset_deg
probabilities
```

如果输出 `NO_VALID`，则应扩大 yaw 搜索范围或终止本次反馈。

### 8.8 模型1自动采集（2026-08-25）

界面默认值已改为：

```text
Probe Range x = 45
Needle Range x = 3
```

模型1采集操作：先点击 `Needle Rotate Left x Deg` 到达 `-3°`，待 Beckhoff Ready 后点击 `Needle Rotate Right 2x Deg`。右转按钮现在执行：

```text
先保存 -3° 静止帧
→ 每次右转 1°
→ 必须先收到 Movement Completed，再收到 Ready
→ 保存当前位置图像和 prior mask
→ 直到 +3°
```

旧的“直接一次右转 2x 度”逻辑已替换。程序强制 `Needle Range x = 3`，避免生成不符合模型1输入定义的序列。

每组输出：

```text
image/model1_center_capture/seq_时间戳/
├── images/m3.png ... p3.png
├── masks/m3.png ... p3.png
└── frame_labels.csv
```

每张 prior mask 根据当前 J0-J3、TCP_P、TCP_U 和未激发针尖运动学在线生成，使用 ±25 px 半宽及针尖侧 6 mm 延伸。若运动学无效、针尖投影在图像外或写入失败，本次采集会停止，不保存缺少有效 mask 的训练帧。

### 8.9 模型1代码验证状态（2026-08-25）

已修复的关键问题：

- `train.py` 中 `Subset` 未正确导入；
- 自动划分改为类别分层；
- 适配灰度/双通道时保留预训练首层卷积信息；
- 增加 CSV 字段、类别、图片和 mask 路径检查；
- 支持类别平衡损失；
- 增加宏平均 F1、各类召回率和混淆矩阵；
- 预测端检查 checkpoint 与 mask 输入是否匹配，并可输出 JSON；
- 添加 `merge_frame_labels.py` 汇总各序列 CSV；
- 添加 `tests/test_model1_center_selector.py`。

验证结果：现有 13 项自动测试全部通过，并用合成的 7 帧图像和 7 张 mask 完成过“训练一轮 → 保存 checkpoint → 加载 → 预测”的闭环测试。临时合成数据已清理。硬件侧仍需连接 Beckhoff 后实测完整 7 帧采集时序。

---

## 9. 模型2：针几何识别模型

模型2尚未完整实现训练代码。当前设计：

输入：

```text
IN_CENTER 图像 + prior mask
```

输出：

```text
未激发针尖 tip 点
针轴 shaft 方向
```

后处理：

```text
活检中心 = 未激发针尖 + 10 mm * 针轴方向
```

用于 pitch feedback。

训练标签需要 LabelMe 人工标注：

```text
tip：未激发针尖
shaft：针轴后方一点
方向：shaft → tip
```

模型2评估指标建议：

```text
针尖误差
针轴角度误差
活检中心估计误差
```

不要把最终 `靶点命中率` 写成模型2本身指标，因为模型2只处理未激发视觉几何；最终命中率属于完整机器人控制实验。

---

## 10. 论文流程图

已生成中文 SVG：

```text
docs/figures/two_model_biopsy_workflow.svg
```

当前图中应表达：

- 原始 TRUS 图像只写“体模数据”；
- 模型1输入 7 帧 yaw 搜索图像，输出 yaw 选择；
- 模型2输入 `IN_CENTER` 图像和 prior mask，输出 tip 点和 shaft 方向；
- 评估指标不写 `yaw定位误差` 和 `靶点命中率`；
- 模型1评价可写“中心判别准确率”；
- 模型2评价写“针尖误差、针轴角度误差、活检中心估计误差”。

用户曾问能否人工编辑 SVG。建议使用：

```text
Inkscape
Adobe Illustrator
VS Code / PyCharm 直接编辑文本
```

Inkscape 官方下载：

```text
https://inkscape.org/download/
```

---

## 11. mini-book 状态

目录：

```text
mini-book/
```

预览命令：

```powershell
.\scripts\preview.cmd
```

会构建并预览 mini-book，通常访问：

```text
http://localhost:8000/
```

常见问题：

1. MyST 自动安装 Node.js 失败或 `node -v` 返回错误；
   - 建议安装官方 Node.js LTS；
2. PDF 构建缺 Typst：
   - 错误：`The typst CLI must be installed to build PDFs with typst`
   - 可用：

```powershell
winget install Typst.Typst
```

---

## 12. 已讨论但不要再沿用的旧想法

以下内容已被用户修正或废弃：

- 不要把模型1设计成单帧 `中心置信度` 输出；
- 不要写“从序列中找最亮最清晰的一张”；
- 不要把模型1评价写成 `yaw定位误差`，因为人工没有真实最佳 yaw 角；
- 不要把模型评价写成 `靶点命中率`，那是完整控制实验指标；
- 不要在探头示意图里乱画 `TCP_U` 坐标系；
- 不要把论文数据写成“体模/临床数据”，本文只用体模。

---

## 13. 下一步建议

短期最重要的是在真实 Beckhoff 和超声设备上验证并开始采集模型1数据：

1. 实机验证自动 yaw sweep：

```text
-3, -2, -1, 0, +1, +2, +3 deg
```

2. 检查每组是否正确保存 7 张图、7 张 prior mask 和 `frame_labels.csv`；
3. 人工逐帧标注：

```text
IN_CENTER / OFF_CENTER
```

4. 用 `merge_frame_labels.py` 汇总逐帧 CSV；
5. 用 `build_sequences.py` 生成序列级训练 CSV；
6. 先采约 100 组检查采集和类别分布，约 300 组训练原型，正式目标约 1000 组完整序列；
7. 用独立采集批次制作验证集和测试集；
8. 训练模型1并检查宏平均 F1、混淆矩阵及 `NO_VALID` 召回率；
9. 离线验证模型1输出到机器人 yaw 动作的映射。

后续接入控制前必须验证：

```text
yaw 正负方向
模型1输出类别到机器人 yaw 动作的映射
模型2针轴方向 shaft → tip
未激发针尖 + 10 mm 的活检中心推算
pitch error 符号
机器人实际执行方向
NO_VALID 时的安全终止逻辑
```

---

## 14. 给新对话的建议开场

可以直接对新 Codex 说：

```text
请完整读取 HANDOFF_SUMMARY_CN.md。
当前项目在 C:\Users\Administrator\Desktop\PROSTATEBIO。
不要重做已有提交，先检查 git status 和最近提交。
当前只优先处理模型1。接下来先实机验证 7 帧 yaw 自动采集、prior mask 和 Beckhoff Ready 时序，再开始批量标注、训练和预测接入。
```
