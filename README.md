# ZED + Livox 実験記録・解析

ZED 2i SVO2 と Livox Mid-360S rosbag2 を記録・再生・解析する。主な環境は Ubuntu、Python 3、ROS 2 Jazzy、ZED SDK 5.4。

## まず見るファイル

| 用途 | ファイル |
| --- | --- |
| 録画・解析設定 | `config.json` |
| 試行順・条件・役割 | `run_order.json`（唯一の参照元） |
| 録画ランチャー | `scripts/record_zed.py` |
| LiDAR人物追跡 | `scripts/track_people.py` |
| ZEDレビュー書き出し | `scripts/export_svo_review.py` |
| ZED/LiDAR比較 | `scripts/compare_zed_lidar.py` |
| SVO2骨格解析 | `scripts/analyze_svo_body_tracking.py` |
| SVO/LiDAR同期再生 | `scripts/replay_zed_lidar.py` |
| 処理の流れ・解析アルゴリズム | [プログラムの解説](docs/analysis_algorithms.md) |
| 軌跡の人手確認手順 | `docs/trajectory_human_review.md` |
| 退避した旧版・過去レポート | [不要なもの/README.md](不要なもの/README.md) |

## 今回使うプログラム

`scripts/` に録画・解析・再生と、その前提となる計画作成・校正の現行プログラムを残している。読み込まれる補助モジュールやHTMLテンプレートも必要なので、一緒に配置する。

| 用途 | `scripts/` 内のファイル |
| --- | --- |
| 録画と試行計画 | `record_zed.py`、`build_run_order.py`、`experiment_plan.py`、`recording_progress.py` |
| LiDAR人物追跡 | `track_people.py`、`assignment.py`、`corridor_frame.py`、`sensor_transform.py` |
| ZED画像・深度・骨格の書き出し | `export_svo_review.py`、`analyze_svo_body_tracking.py` |
| 比較・融合・軌跡評価 | `compare_zed_lidar.py`、`fuse_zed_lidar.py`、`plot_fused_trajectory.py`、`evaluate_tracking.py`、`render_sensor_review.py` |
| 録画の時計・設置状態の確認 | `audit_recordings.py` |
| 同期再生・点群変換 | `replay_zed_lidar.py`、`playback_timeline.py`、`convert_livox_custom.py` |
| 軌跡・点群・融合骨格の再生 | `replay_tracks_csv.py`、`tracking_replay.html`、`replay_tracking_rviz.py`、`replay_fused_trajectory_rviz.py` |
| 書き出し済み骨格の確認・再生 | `build_walk_skeleton_replay.py`、`walk_skeleton_replay_template.html`、`assess_skeleton_export.py`、`compare_approach_tracking.py` |
| 設置時の座標・外部校正 | `estimate_corridor_frame.py`、`calibrate_zed_lidar_planes.py`、`calibrate_zed_lidar_aruco.py` |

骨格再生関連の後処理には、対象録画用の `bundle.json`・画像assets等を揃えた `--bundle-dir` が必要。再生ビルダーが読むテンプレート名は、そのフォルダー内の `skeleton_replay_template.html`。`scripts/walk_skeleton_replay_template.html` が現行テンプレートである。過去の録画003用bundleは過去レポートと一緒に退避しており、別録画には流用しない。

試行計画の元Excelは `inputs/`、設置依存の校正JSONは `tracking/` に保持している。過去のCSV・比較HTML等も解析入力として参照されるため、`tracking/` 内では単独の解析レポートMarkdownだけを退避した。

## 基本コマンド

録画前に `config.json` と `run_order.json` を確認する。試行計画のExcelを更新したときは録画前に作り直す。

```bash
python3 scripts/build_run_order.py
python3 scripts/record_zed.py
```

各グループ（G01〜G18）は **18試行×4セット＝72試行**。`ubuntu_72_trials/` と照合し、従来の1〜54試行の順番・条件・役割・教示は維持している。55〜72試行は未使用のC1またはC2で、録画前画面から最初から選択でき、54試行の後も通常どおり進む。画面には通し番号（/72）、セット、用紙名、用紙内番号、条件IDを表示する。

録画時の参照元は引き続き `run_order.json`。計画JSONは既存の全72行と計画ハッシュを保持しているため、同じ出力先の保存済み進捗から再開できる。旧進捗の追加セット有効・辞退フラグは選択範囲を制限しない。54試行でQを押した場合は中断となり、55〜72試行は未完了として次回の再開対象になる。元資料とJSON内の「基本54＋追加18」「任意」という区分は資料照合用に残している。

全18グループの照合結果は [72試行への変更確認](reports/20261022_run_order_72_audit.md) を参照する。起動方法は従来どおり。

保存済みbagの追跡例:

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/track_people.py \
  /data/experiment_data/recordings/<group>/recording_<NNN>/lidar \
  --topic /livox/lidar
```

レビュー画像・深度を書き出す場合は、ZED SDK対応の外部PCで実行する。SVO2はそのPCのローカルから読み、出力は共有領域の新規フォルダーにする。

```powershell
python "\\tsclient\_data_experiment_data\scripts\export_svo_review.py" `
  "C:\data\recording.svo2" `
  --output "\\tsclient\_data_experiment_data\recording_svo_review" `
  --every-seconds 0.5 --include-depth
```

書き出し後の比較例:

```bash
python3 scripts/compare_zed_lidar.py \
  --review-dir /data/experiment_data/recording_svo_review \
  --tracks tracking/<recording>_tracks.csv \
  --lidar-time-offset-s <録画ごとの確認値>
```

詳しい引数は各スクリプトの `--help` を参照する。
点群処理・人物ID対応・ZED骨格抽出・位置融合・評価指標の仕組みは[処理と解析アルゴリズムの解説](docs/analysis_algorithms.md)を参照する。
プログラム軌跡を映像で確認し、独立真値がある場合に評価する手順は[軌跡の人手確認手順](docs/trajectory_human_review.md)を参照する。

外部Windows PCのSDK 5.5.1では、`NEURAL / BODY_34`を固定し、`MEDIUM / GEN_2`を第一候補として5種類のモデル・世代を比較する準備を追加した。[比較条件と実行コマンド](reports/20261022_sdk551_body_model_comparison/README.md)を参照。`analyze_svo_body_tracking.py`の`--body-model`・`--model-generation`で明示指定でき、`--require-generation-readback`は有効な世代を読み戻せない実行を拒否する。従来の指定なしのモデルはACCURATEのまま、世代はSDK既定値に従うため、再現比較では必ず明示する。録画設定・運用中の校正・過去結果は変更していない。

## 座標と校正

- `x,y,z`: LiDAR元座標。`level_x/y/z`: 重力水平化座標。`corridor_x/y/z`: 廊下座標。`zed_x/y/z`: ZED IMAGE座標（右・下・前）。位置と深度はm、ZED SDK既定値はmmの場合がある。
- 確定人物検出はCSVの `status=confirmed` かつ `detected=1` を使う。候補・予測を実検出に数えない。
- `tracking/lidar_to_zed_left_eye.json` は床・平行壁に基づく部分校正で、廊下方向の並進は初期取り付け値から保持している。精密な完全6自由度校正として扱わない。
- `tracking/corridor_frame_livox_level.json` と重力校正は設置依存。移設後は取り直す。録画ごとに時刻offsetを確認し、別録画へ固定値を流用しない。
- 投影ずれが出たときは時刻対応と外部校正を確認する。見た目だけで校正JSONを変更しない。
- ZED画像の前景比較は移動領域候補であり、Body Trackingや意味的な人物検出ではない。3D差も真値に対する測位誤差ではない。

廊下座標、ArUco校正、SVO骨格、LiDAR/ZED融合、再生の引数は各スクリプトを参照する。過去の検証記録は [退避した解析レポート一覧](不要なもの/reports/INDEX.md) にある。HTML出力を共有するときは同名の `_frames/` フォルダーも含める。

## 共有データの扱い

- Linux共有先は `/data/experiment_data/`、Windows側は `\\tsclient\_data_experiment_data\`。
- 録画原本と既存メタデータは変更しない。新規録画はランチャーで採番し、解析出力は `tracking/`、`reports/` または新規専用フォルダーへ保存する。
- ZED SDK用スクリプトは指定がある場合 `/data/experiment_data/scripts/` に置く。共有先へ書けない場合は権限を回避せず、リポジトリ内で準備してコピーが必要と伝える。
- 大容量SVO2やbagを無断で複製しない。出力先の既存ファイルを上書きしない。

## 現状の精度上の注意

2026-10-13の1人往復録画では、確定LiDARトラックID 45が約12.92 mまで確認された。製品の40 m測距仕様は人物検出距離を保証しない。記録ごとの結果と限界は `reports/` に保存する。

2026-10-22に、録画003の全318点群で現行処理と学習済みPointPillarsを比較した。近中距離の経路はおおむね一致したが、学習済みモデルで遠方・折返しの連続観測や位置変動が改善した結果ではない。[比較結果と映像付き再生](reports/20261022_pretrained_lidar_comparison/README.md)を参照する。これは1人往復の検証であり、2人追越し・関節位置の精度保証ではない。

## 開発・確認

主要処理は `scripts/`、設定と試行計画はルートJSONにある。開発用テストと独立した校正GUIは `不要なもの/` へ退避した。再利用するときは [退避一覧と復元方法](不要なもの/README.md) を参照し、元の配置へ戻す。テストは依頼時に実行する。

2026-10-22の整理では、移動前後のファイル内容と残すファイルの参照関係を確認した。テスト・実機操作は行っていない。共有先 `/data/experiment_data/` のプログラム・データも変更していない。

実機のZED、Livox、ROS 2、RViz操作はソフトウェアだけの確認と分けて扱う。
