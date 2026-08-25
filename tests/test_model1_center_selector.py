import csv
import tempfile
import unittest
from pathlib import Path

from model1_center_selector.build_sequences import build_sequence_csv
from model1_center_selector.label_rules import NO_VALID_CLASS, select_center_target
from model1_center_selector.merge_frame_labels import merge_frame_csvs
from model1_center_selector.train import stratified_split_indices


class Model1LabelPipelineTests(unittest.TestCase):
    def test_selection_priority_prefers_zero_then_negative(self):
        decision = select_center_target([
            "OFF_CENTER", "OFF_CENTER", "IN_CENTER", "OFF_CENTER",
            "IN_CENTER", "OFF_CENTER", "OFF_CENTER",
        ])
        self.assertEqual(decision.target_yaw_offset_deg, -1)

        labels = ["OFF_CENTER"] * 7
        labels[3] = "IN_CENTER"
        self.assertEqual(select_center_target(labels).target_yaw_offset_deg, 0)

    def test_all_off_center_becomes_no_valid(self):
        decision = select_center_target(["OFF_CENTER"] * 7)
        self.assertEqual(decision.target_class, NO_VALID_CLASS)
        self.assertIsNone(decision.target_yaw_offset_deg)

    def test_build_sequence_csv_orders_all_seven_yaws(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            frame_csv = root / "frames.csv"
            sequence_csv = root / "sequences.csv"
            with frame_csv.open("w", encoding="utf-8", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=(
                    "sequence_id", "yaw_offset_deg", "image_path", "mask_path", "center_label"
                ))
                writer.writeheader()
                for yaw in (3, 1, -3, 0, -1, 2, -2):
                    writer.writerow({
                        "sequence_id": "seq_1", "yaw_offset_deg": yaw,
                        "image_path": f"image/{yaw}.png", "mask_path": f"mask/{yaw}.png",
                        "center_label": "IN_CENTER" if yaw == -1 else "OFF_CENTER",
                    })
            build_sequence_csv(frame_csv, sequence_csv)
            with sequence_csv.open("r", encoding="utf-8", newline="") as f:
                row = next(csv.DictReader(f))
            self.assertEqual(row["target_name"], "YAW_M1")
            self.assertEqual(row["image_m3"], "image/-3.png")
            self.assertEqual(row["image_p3"], "image/3.png")

    def test_merge_capture_csvs_rejects_no_data_and_merges_sequences(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            for sequence_id in ("seq_a", "seq_b"):
                folder = root / sequence_id
                folder.mkdir()
                with (folder / "frame_labels.csv").open("w", encoding="utf-8", newline="") as f:
                    writer = csv.DictWriter(f, fieldnames=(
                        "sequence_id", "yaw_offset_deg", "image_path", "mask_path", "center_label"
                    ))
                    writer.writeheader()
                    writer.writerow({
                        "sequence_id": sequence_id, "yaw_offset_deg": "-3",
                        "image_path": "a.png", "mask_path": "a_mask.png", "center_label": "OFF_CENTER",
                    })
            output = root / "merged.csv"
            self.assertEqual(merge_frame_csvs(root, output), 2)

    def test_stratified_split_keeps_train_and_validation_nonempty(self):
        rows = [{"target_class": str(index % 2)} for index in range(10)]
        train, val = stratified_split_indices(rows, 0.2, 42)
        self.assertEqual(len(set(train).intersection(val)), 0)
        self.assertEqual(len(train) + len(val), len(rows))
        self.assertGreater(len(train), 0)
        self.assertGreater(len(val), 0)


if __name__ == "__main__":
    unittest.main()
