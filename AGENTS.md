# プロジェクト作業ルール

ZED 2i / Livox Mid-360S の歩行実験録画・解析リポジトリ。主環境は Ubuntu、Python 3、ROS 2 Jazzy、ZED SDK 5.4。

## 基本

- 試行順・条件・役割の唯一の参照元は `run_order.json`。Excelを変えたら録画前に `scripts/build_run_order.py` で再生成し、スクリプトへ重複記載しない。
- 録画原本と既存メタデータは読み取り専用として扱う。依頼なしに共有録画を削除・移動・改名・上書きしない。新しい出力名を使い、原本と解析結果を混在させない。
- 作業前に関連コードとREADMEを確認する。録画・設定・校正へ影響する変更はREADMEまたはレポートへ反映する。
- 実機（ZED、Livox、ROS 2、RViz）の操作は明記する。テストは依頼された場合だけ実行する。

## データと環境

- 共有データ: Linux `/data/experiment_data/`、Windows `\\tsclient\_data_experiment_data\`。
- ZED SDK用スクリプトはユーザー指定により `/data/experiment_data/scripts/` に置く。SVO2は外部PCのローカルから読み、出力は共有領域の新規専用フォルダーへ保存する。
- `/data/experiment_data` に書けない場合、権限を回避せずリポジトリ内で作業し、コピーが必要と伝える。大容量録画を無断複製しない。
- PowerShellの継続記号はバッククォート。Python依存は実行対象と同じ環境で `python -m pip` を使う。

## 座標・解析上の注意

- 人物の確定検出はCSVの `status=confirmed` かつ `detected=1`。未確定候補を人数や成功検出に数えない。
- LiDAR元座標 `x,y,z`、水平化後 `level_*`、廊下座標 `corridor_*`、ZED IMAGE座標 `zed_*` を混同しない。位置・深度は通常m、ZED SDKの既定値はmmの場合がある。
- 重力、床高、廊下座標は設置依存。センサー移動後は記録・再校正し、別設置へ流用しない。
- `tracking/lidar_to_zed_left_eye.json` は床・平行壁による部分校正で、廊下方向の並進は初期値由来。完全な6自由度精度を保証しない。見た目に合わせてJSONを直接調整しない。
- 時計差・offsetは録画ごとに確認する。過去録画の値を固定適用しない。静的点群との時刻近接だけで同期精度を証明しない。
- ZED前景比較は画像・深度の移動領域候補であり、Body Trackingや意味的な人物認識ではない。比較HTMLは対応する `_frames/` と一緒に保つ。
- 2026-10-13の1人往復では確定LiDARトラックID 45が約12.92 mまで検出された。製品の40 m測距仕様は人物追跡距離を意味しない。

## 主な入口

- 録画: `scripts/record_zed.py`、設定: `config.json`。
- LiDAR追跡: `scripts/track_people.py`。
- SVO2レビュー書き出し: `scripts/export_svo_review.py`（外部ZED SDK PC）。
- SVO/LiDAR再生: `scripts/replay_zed_lidar.py`。
- ZED/LiDAR比較: `scripts/compare_zed_lidar.py`。
- ArUco校正候補: `scripts/calibrate_zed_lidar_aruco.py`。未使用の独立データで確認するまで適用しない。
- 手順・設定の詳細はルート `README.md`、退避した過去の解析記録は `不要なもの/reports/INDEX.md` を参照。新規の解析出力先は引き続き `reports/` を使用する。
