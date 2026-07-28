# 前列腺超声活检针识别与机器人反馈控制项目交接摘要

最后更新：2026-07-29  
当前工作区：`C:\Users\Administrator\Desktop\robio`  
主要机器人程序仓库：`C:\Users\Administrator\Desktop\robio\PROSTATEBIO`

> 用途：把整个工作区复制到另一台电脑后，将本文件交给新的 Codex 对话读取，即可继续当前项目。不要依赖原电脑的绝对路径或直接复制虚拟环境。

---

## 1. 项目总体目标

项目研究前列腺超声图像中的活检针识别，以及基于视觉识别结果的早期方向反馈控制。

当前研究分成两部分：

1. 尸体超声数据：
   - 用于证明针存在检测、针尖定位和针方向估计的可行性；
   - 已训练 ResNet18 多任务 baseline；
   - 可用于 RoBio preliminary / cadaveric feasibility paper。
2. 前列腺体模实验：
   - 新采集体模穿刺图像；
   - 利用机器人运动学生成针尖、针向量和四角先验框；
   - 训练一个专门用于体模实验的模型；
   - 模型识别视觉针向量后，计算实测 Needle Pitch；
   - 在针进入前列腺之前，通过离散、小步、人工许可的 Pitch 反馈进行方向调整。

反馈控制目前只考虑：

```text
Pitch Error = Target Pitch - Measured Pitch
```

暂不把 `cross_track_error` 和 `along_error` 放入控制器。

Yaw 在第一版视觉反馈中保持不变，只调整 Pitch。

---

## 2. 工作区和 Git 仓库

工作区根目录：

```text
C:\Users\Administrator\Desktop\robio
```

其中 `PROSTATEBIO` 是独立的嵌套 Git 仓库：

```text
C:\Users\Administrator\Desktop\robio\PROSTATEBIO
```

机器人代码修改和提交都在 `PROSTATEBIO` 仓库中进行，不是在外层仓库中进行。

当前分支：

```text
main
```

截至 2026-07-29，工作区已确认干净。最近提交：

```text
d66c14e Require manual permit for feedback insertion
dc42cd4 Add safe needle pitch feedback controller
4df36ca Add kinematic prior boxes to ultrasound captures
a781880 Save ultrasound frames without needle kinematics
7e2632e Add needle kinematics to saved ultrasound images
54721f0 Fix needle yaw and pitch control flow
d5dfe56 revert: delta_j2 -2
1be8f0c add: some change
```

---

## 3. Python 环境

原电脑确认使用：

```text
Python 3.11.9
```

外层工作区虚拟环境：

```text
robio\.venv
```

曾安装：

```text
torch 2.13.0 CPU
torchvision 0.28.0
Pillow 12.3.0
```

`PROSTATEBIO` 目录内还存在：

```text
PROSTATEBIO\.venv311
```

换电脑时不要直接复制并继续使用 `.venv` 或 `.venv311`。虚拟环境通常包含旧电脑的绝对路径，跨电脑后可能失效。

应先安装同一版本 Python 3.11.9，然后重新创建环境：

```powershell
py -3.11 -m venv .venv
.venv\Scripts\python.exe -m pip install --upgrade pip
.venv\Scripts\python.exe -m pip install -r requirements_baseline.txt
```

机器人程序依赖应根据：

```text
PROSTATEBIO\requirements.txt
```

重新安装。不要改变 Python 大版本。

---

## 4. 数据目录

### 4.1 尸体数据

目录：

```text
needle_detection
```

数据情况：

- PNG：593 张；
- LabelMe JSON：593 个；
- 有针图像：404 张；
- 无针图像：189 张；
- 图像尺寸：406×420；
- 比例：`66 mm / 420 px = 0.1571 mm/px`。

人工标注：

- 无针：`shapes` 为空；
- 有针：
  - `tip`：针尖；
  - `axis`：针轴后方一点。

方向统一为：

```text
针尾/axis → 针尖/tip
```

关键文件：

```text
needle_detection/labels.csv
needle_detection/sequences_by_mtime.csv
needle_detection/splits/train.txt
needle_detection/splits/val.txt
needle_detection/splits/test.txt
needle_detection/splits/sequence_level_summary.csv
```

数据按扫描序列划分，不能把同一次扫描中的连续帧随机拆到 train/val/test。

### 4.2 体模数据

原始体模图片目录：

```text
image
```

曾将其中 5539 张图批量裁剪为保留顶部 270 像素：

```text
image_cropped_top270
```

注意：旧体模图与当前机器人实时保存图的尺寸、裁剪方式可能不一致。训练前必须确定最终运行时使用的固定尺寸，不要直接混合不一致的数据。

---

## 5. 尸体数据 baseline

训练脚本：

```text
train_multitask_baseline.py
```

模型：

- ResNet18 backbone；
- `cls_head`：是否有针；
- `tip_head`：针尖归一化坐标；
- `angle_head`：`sin_theta, cos_theta`。

训练命令示例：

```powershell
.venv\Scripts\python.exe train_multitask_baseline.py --data needle_detection --epochs 100 --batch-size 16 --lr 1e-4 --image-size 224 --output runs/sequence_resnet18_baseline
```

主要测试结果：

### From-scratch ResNet18

```text
Accuracy       88.5%
Precision      94.3%
Recall         87.7%
Specificity    90.0%
Tip error      30.58 px / 4.80 mm
Angle error    1.47 deg
```

### ImageNet-pretrained ResNet18

```text
Accuracy       88.5%
Precision      96.1%
Recall         86.0%
Specificity    93.3%
Tip error      30.65 px / 4.82 mm
Angle error    1.82 deg
```

关键结果文件：

```text
runs/sequence_resnet18_baseline/best.pt
runs/sequence_resnet18_baseline/log.csv
runs/sequence_resnet18_pretrained/best.pt
runs/sequence_resnet18_pretrained/log.csv
```

可视化脚本：

```text
visualize_predictions.py
```

论文草稿：

```text
robio_needle_perception_draft.docx
```

生成脚本：

```text
build_robio_paper_docx.py
```

---

## 6. PROSTATEBIO 中已完成的关键代码修改

### 6.1 TCP_P 到 TCP_U 的变换

`transform_point_p_to_tcp_u` 已修正，不再使用：

```text
tcp_e_in_ultrasound_zero_deg
```

而是使用两个相对于 TCP_E 定义的固定 TCP：

```python
T_e_u = pose_to_matrix(tcp_u_definition_pose)
T_e_p = pose_to_matrix(tcp_p_definition_pose)
T_u_p = np.linalg.inv(T_e_u) @ T_e_p
```

点变换：

```python
point_u = T_u_p @ point_p_homogeneous
```

向量变换只使用旋转：

```python
vector_u = T_u_p[:3, :3] @ vector_p
```

### 6.2 Yaw/Pitch 控制

- Yaw/Pitch 输入框只连接 `returnPressed`；
- 按 Enter 后重新计算并执行 `Apply Increment (All)`；
- 不再同时使用 `editingFinished`；
- `+/-` 按钮修改目标角度后会重新计算并立即执行；
- Needle Right 2x 已改为一次性目标运动，不再拆成 0.5° 定时小步；
- 逆运动学失败时不会继续使用旧的 delta J0-J3；
- `Adjust Needle Dir` 中硬编码 `Yaw=1° / Pitch-2°` 的实验补偿按用户要求保留。

### 6.3 图像保存

`Start Image Saving`：

- 每 300 ms 保存一张；
- 保存目录：

```text
PROSTATEBIO/image/Realtime_Capture_Interval_300ms_YYYYMMDD_HHMMSS
```

- 文件名：

```text
0000_E(...)_TipU(...)_VecU(...)_Box(...).png
```

其中：

- `E`：TCP_E 位姿；
- `TipU`：运动学针尖在 TCP_U 下的位置；
- `VecU`：运动学针向量在 TCP_U 下的单位向量；
- `Box`：运动学四角先验框。

如果针运动学数据无效：

```text
TipU、VecU、Box 字符留空，但图像仍保存
```

注意：

- `Save Single Image` 当前只是普通文件对话框保存，不带这些运动学元数据；
- 连续保存需要有效 TCP_E 位姿，否则该帧会跳过；
- 文件名很长，正式实验前应测试 Windows 是否能正常保存，避免路径长度问题；
- 当前没有自动生成 LabelMe JSON 或训练真值。

### 6.4 运动学 TipU 和 VecU

当前 J0-J3 计算：

```python
delta_j0 = current_j0 - RESET_J0
delta_j1 = current_j1 - RESET_J1
delta_j2 = current_j2 - RESET_J2
delta_j3 = current_j3 - RESET_J3

joint_values = [
    delta_j0,
    delta_j1,
    delta_j2 - delta_j1,
    delta_j3,
]
```

变量名按用户要求保持：

```text
delta_j2
```

不要改成 `delta_j2_motor`。

针尖和针向量：

```python
tip_p = robot.get_tip_of_needle(joint_values.copy())
vector_p = robot.get_needle_vector(joint_values.copy())
```

然后变换到 TCP_U。

---

## 7. 超声图像与 TCP_U 坐标关系

用户给定的标定关系：

```text
图像 u 正方向 = TCP_U 的 -z 方向
图像 v 正方向 = TCP_U 的 -x 方向
TCP_U y 垂直于超声成像平面
```

比例：

```python
MM_PER_PIXEL = 66.0 / 420.0
```

固定投影原点：

```python
TCP_U_ORIGIN_U_PX = 406.0
TCP_U_ORIGIN_V_PX = 420.0 + 10.0 / MM_PER_PIXEL
```

用户明确要求：

> 裁剪滑块改变后，不要根据 left crop / top crop 自动修正投影原点；用户会重新调整标定。

针尖投影：

```python
u = u0 - tip_u_z / MM_PER_PIXEL
v = v0 - tip_u_x / MM_PER_PIXEL
```

针向量投影：

```python
du = -vector_u_z / MM_PER_PIXEL
dv = -vector_u_x / MM_PER_PIXEL
```

向量投影不需要加图像原点。

---

## 8. 四角运动学先验框

代码：

```text
PROSTATEBIO/ui/ultrasound_tab.py
```

逻辑：

1. 将 `TipU` 投影为图像针尖；
2. 将 `VecU` 投影为图像二维方向；
3. 只有投影针尖在图像内才生成 Box；
4. `VecU` 定义为针尾指向针尖，因此从针尖沿 `-VecU` 找针尾侧；
5. 从针尖沿针尾方向延伸到图像边界；
6. 在针尖和边界中心点处沿法向各扩展 10 px；
7. 得到四角；
8. 越界角使用 `np.clip` 截到图像边界；
9. 针尖不在图像内、向量退化或中心线不与图像相交时，Box为空。

四角框不是人工真值，它表示：

```text
根据机器人运动学，针理论上可能出现的区域
```

训练时应将四角框生成二值 mask，作为模型第二输入通道。

不要把四角框当作真实针标签。

---

## 9. 自动标注的实际状态

当前只完成了“自动运动学预标注”：

```text
TipU
VecU
Box
```

尚未自动生成：

```text
视觉真实针尖
视觉真实针轴
IN_PLANE / OUT_OF_PLANE / NO_NEEDLE
feedback_valid
LabelMe JSON
针分割 mask
病灶点编号
```

这些仍需人工标注。

推荐人工标签：

```text
tip：真实针尖
shaft：针轴后方一点
方向：shaft → tip
class：IN_PLANE / OUT_OF_PLANE / NO_NEEDLE / PARTIAL_NEEDLE
feedback_valid：只有可靠平面内针为 true
```

针尖不可见时不要根据运动学框猜针尖。

---

## 10. 体模模型训练方案

推荐输入：

```text
通道0：超声灰度图
通道1：运动学四角先验 mask
```

即：

```text
[Batch, 2, Height, Width]
```

推荐多任务输出：

1. 针分割 mask；
2. 针尖 heatmap；
3. `VALID_FOR_FEEDBACK / INVALID_FOR_FEEDBACK`；
4. 可选直接输出 `VecUV`。

建议第一版使用轻量 U-Net、ResNet18 encoder 或 MobileNetV3，不要先上很大的模型。

训练时：

- 几何增强必须同步作用于图像、先验 mask、针尖、针轴；
- 亮度、Gamma、噪声等只作用于超声图像；
- 随机扰动运动学框；
- 一部分样本使用全零先验 mask，防止模型过度依赖运动学；
- train/val/test 必须按整次穿刺序列划分；
- 同一停止位置的连续短帧不能拆到不同集合。

反馈安全指标重点关注：

```text
模型预测可以反馈，但实际上针已经离面
```

也就是要优先降低“假有效率”，不能只看总 Accuracy。

---

## 11. 从视觉 VecUV 计算实测 Needle Pitch

项目 Yaw/Pitch 定义：

```python
vx = -sin(yaw) * cos(pitch)
vy =  cos(yaw) * cos(pitch)
vz =  sin(pitch)
```

当：

```text
Yaw = 0
Pitch = 0
Roll = 0
```

针向量在 TCP_P 下是：

```text
[0, 1, 0]
```

模型输出二维向量：

```text
VecUV = [du, dv]
```

转换为 TCP_U 超声平面向量：

```python
vector_u = [-dv, 0, -du]
```

再从 TCP_U 转回 TCP_P：

```python
R_u_p = T_u_p[:3, :3]
vector_p = R_u_p.T @ vector_u
```

实测 Pitch：

```python
measured_pitch = atan2(
    vector_p_z,
    sqrt(vector_p_x**2 + vector_p_y**2)
)
```

目标 Pitch 由目标 TCP_P 向量使用同样物理定义计算：

```python
target_pitch = atan2(
    target_vector_p_z,
    sqrt(target_vector_p_x**2 + target_vector_p_y**2)
)
```

误差：

```python
pitch_error = target_pitch - measured_pitch
```

模型针向量必须统一为针尾到针尖。若模型输出反向，需要翻转。

成立前提：

```text
针位于超声成像平面内
```

如果模型判断针离面，反馈必须终止，不能继续用二维投影计算 Pitch。

---

## 12. Pitch Feedback Controller

核心代码：

```text
PROSTATEBIO/core/needle_pitch_feedback.py
```

测试：

```text
PROSTATEBIO/tests/test_needle_pitch_feedback.py
```

已实现：

- UV → TCP_U → TCP_P → Measured Pitch；
- 目标向量 → Target Pitch；
- `Target Pitch - Measured Pitch`；
- 离散 PID；
- 默认参数：

```python
kp = 0.3
ki = 0.0
kd = 0.0
deadband_deg = 1.0
max_pitch_step_deg = 0.2
max_feedback_steps = 10
min_in_plane_confidence = 0.8
```

- 第一版实际相当于离散 P 控制；
- 只输出 `delta_pitch_deg`，没有 Yaw 输出；
- 模型未就绪时拒绝启动；
- 离面、置信度不足、无效向量或超过步数时终止；
- 终止时清空 PID 状态和未用进针许可；
- 控制器目前不加载模型，也不直接发送电机命令。

测试命令：

```powershell
..\.venv\Scripts\python.exe -m unittest tests.test_needle_pitch_feedback -v
```

截至提交 `d66c14e`，6 项测试通过。

---

## 13. 单次人工进针许可

界面：

```text
Flexible needle steering
```

新增按钮：

```text
Permit One Feedback Insertion
```

设计原则：

```text
每点击一次，只允许进针一次
```

具体行为：

1. 模型未加载时按钮禁用；
2. 必须先有有效、平面内的模型反馈结果；
3. 点击一次只发放一个布尔许可；
4. 许可不能累计；
5. 执行器运动前必须消费许可；
6. 消费后下一次必须重新识别并重新点击；
7. 离面后即使已有未使用许可，也立即清除；
8. 超过最大反馈步数后拒绝新许可。

目前模型尚未训练完成，因此：

- 按钮默认禁用；
- 尚未接入自动模型推理；
- 尚未把许可消费动作接到真实固定步长 J3 进针；
- 不会自动驱动电机。

未来完整流程应为：

```text
模型识别平面内针
→ 计算 Measured Pitch
→ 计算 Pitch Error
→ 执行受限 ΔPitch
→ 重新识别确认有效
→ 人工点击 Permit One Feedback Insertion
→ 执行器消费许可
→ J3 固定进针一步
→ 再次采图
```

---

## 14. 反馈控制的临床/实验约束

反馈控制只应发生在穿刺早期：

```text
针开始出现在超声平面内
→ 可能尚未进入前列腺
→ 进行有限次数 Pitch 调整
```

一旦可能进入前列腺：

```text
锁定 Yaw/Pitch
```

之后不再横向改变方向，只允许：

```text
沿针轴进针
停止
人工处理
```

原因：针进入前列腺后继续改变 Yaw/Pitch 可能造成组织横向扫动和损伤。

未来需要设置：

```text
固定每步进针量
最大反馈步数
最大累计反馈进针距离
方向锁定深度
离面/低置信度终止
人工停止
```

这些实际参数尚未通过体模实验确定。

---

## 15. 12个病灶点体模采集建议

计划：机器人穿刺体模的 12 个病灶点并采集针图像。

建议每个病灶点单独开始和停止保存：

```text
lesion_01 → Start Image Saving → 完成 → Stop
lesion_02 → Start Image Saving → 完成 → Stop
...
lesion_12
```

必须记录：

```text
时间戳文件夹 ↔ lesion编号 ↔ puncture编号
```

正式实验前短测试：

1. 启动超声；
2. 确认 TCP_E 位姿有效；
3. 确认 Current J0-J3 正常更新；
4. 确认 TCP_P/TCP_U definition 已加载；
5. 点击 `Start Image Saving`；
6. 保存 3～5 秒后停止；
7. 确认图片实际写入；
8. 检查文件名中的 `TipU/VecU/Box`；
9. 随机打开图片检查裁剪和画面；
10. 观察是否有 `Real-time image saving failed`。

需要有意识采集：

- 无针；
- 针刚出现；
- 清晰平面内针；
- 轻微离面；
- 明显离面；
- 短针段；
- 针尖不可见；
- 针靠近图像边缘；
- 运动学框准确与偏移；
- 强反射伪影；
- 多个初始 Pitch；
- 多个进针深度；
- 多次独立穿刺，而不是只积累相邻重复帧。

---

## 16. 论文定位

目前可写：

```text
RoBio preliminary / cadaveric feasibility paper
```

不应声称：

```text
clinical deployment-ready
```

尸体 baseline 能证明：

- 多任务针感知可行；
- sequence-level split 降低数据泄漏；
- 分类和方向估计表现较好；
- 针尖定位仍需提升。

主要限制：

- 单尸体；
- 数据量有限；
- 无真实患者数据；
- 针尖误差约 4.8 mm；
- 尚无已验证的实时闭环；
- 体模模型尚未训练；
- 离面判断尚需专门数据。

---

## 17. 关键文件索引

外层训练与论文：

```text
HANDOFF_SUMMARY_CN.md
train_multitask_baseline.py
visualize_predictions.py
build_robio_paper_docx.py
requirements_baseline.txt
robio_needle_perception_draft.docx
needle_detection/
runs/
image/
image_cropped_top270/
```

机器人程序：

```text
PROSTATEBIO/ui/ultrasound_tab.py
PROSTATEBIO/ui/flexible_needle_tab.py
PROSTATEBIO/ui/beckhoff_tab.py
PROSTATEBIO/ui/left_panel.py
PROSTATEBIO/core/needle_pitch_feedback.py
PROSTATEBIO/tests/test_needle_pitch_feedback.py
PROSTATEBIO/kinematics/prostate_biopsy_robot_kinematics.py
```

---

## 18. 换电脑后的恢复步骤

1. 完整复制 `robio` 工作区，包括隐藏的 `.git` 目录；
2. 不要依赖复制过来的 `.venv`；
3. 安装 Python 3.11.9；
4. 重新创建虚拟环境并安装依赖；
5. 在 `PROSTATEBIO` 中检查：

```powershell
git status
git log --oneline -8
```

6. 确认最新提交至少包含：

```text
d66c14e
dc42cd4
4df36ca
```

7. 运行反馈控制测试；
8. 启动程序前确认 TCP/ADS/超声采集依赖；
9. 先做离线和无电机测试；
10. 将本文件交给新对话读取。

建议给新 Codex 的第一句话：

```text
请完整读取 HANDOFF_SUMMARY_CN.md。PROSTATEBIO 是独立 Git 仓库。
先检查当前工作区和最近提交，不要重做已经完成的修改。
继续处理体模超声针识别、自动先验框和离散 Pitch 反馈控制。
```

---

## 19. 当前下一步

短期优先级：

1. 完成 12 个病灶点的体模图像采集；
2. 整理病灶点、穿刺序列和文件夹映射；
3. 将长文件名中的运动学信息整理为 manifest/JSONL；
4. 人工标注视觉 `tip/shaft/class/feedback_valid`；
5. 按序列划分 train/val/test；
6. 训练两通道多任务体模模型；
7. 先离线验证 VecUV 和离面状态；
8. 再把模型接到 `NeedlePitchFeedbackController`；
9. 最后连接固定步长、人工单次许可的 J3 进针。

任何自动闭环上线前，都必须先验证：

```text
VecUV方向
TCP_U/TCP_P变换
Measured Pitch符号
Pitch Error符号
ΔPitch执行方向
离面终止
单次许可不可重复消费
```
