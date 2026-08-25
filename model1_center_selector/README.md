# Model 1: Ultrasound Center Selector

This module implements the first baseline for the yaw-centering model.

## Task

Input: seven TRUS frames collected at fixed yaw offsets:

```text
-3, -2, -1, 0, +1, +2, +3 deg
```

Output: one of eight classes:

```text
YAW_M3, YAW_M2, YAW_M1, YAW_0, YAW_P1, YAW_P2, YAW_P3, NO_VALID
```

The selected yaw is the nearest `IN_CENTER` frame to `0 deg`.
If positive and negative offsets are equally near, the negative direction is preferred.

Priority:

```text
0, -1, +1, -2, +2, -3, +3, NO_VALID
```

## Per-frame label CSV

Create a CSV with one row per frame:

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

Allowed labels:

```text
IN_CENTER, OFF_CENTER
```

The acquisition UI creates one `frame_labels.csv` inside every `seq_*` folder.
After labeling those files, merge them with:

```powershell
.\.venv311\Scripts\python.exe -m model1_center_selector.merge_frame_labels `
  --capture-root image/model1_center_capture `
  --output-csv model1_center_frame_labels.csv
```

## Build sequence-level labels

```powershell
.\.venv311\Scripts\python.exe -m model1_center_selector.build_sequences `
  --frame-csv model1_center_frame_labels.csv `
  --output-csv model1_center_sequences.csv
```

## Train

Without prior masks:

```powershell
.\.venv311\Scripts\python.exe -m model1_center_selector.train `
  --train-csv model1_center_sequences.csv `
  --root-dir . `
  --output runs/model1_center_selector/best.pt
```

With prior masks:

```powershell
.\.venv311\Scripts\python.exe -m model1_center_selector.train `
  --train-csv model1_center_sequences.csv `
  --root-dir . `
  --use-masks `
  --output runs/model1_center_selector/best.pt
```

For imbalanced classes, add `--class-balanced-loss`. For a trustworthy
experiment, provide `--val-csv` built from separate capture sessions instead
of relying on the automatic split. ImageNet pretraining is enabled by default;
use `--no-pretrained` only for offline smoke tests or ablation experiments.

## Predict

Images must be passed in yaw order:

```text
-3, -2, -1, 0, +1, +2, +3
```

```powershell
.\.venv311\Scripts\python.exe -m model1_center_selector.predict `
  --checkpoint runs/model1_center_selector/best.pt `
  --images m3.png m2.png m1.png zero.png p1.png p2.png p3.png `
  --masks m3_mask.png m2_mask.png m1_mask.png zero_mask.png p1_mask.png p2_mask.png p3_mask.png
```

