# 再現・追加評価の手順

作業開始時の基準コミットは `37aa52a`。本作業の成果物ディレクトリ名は開始日を維持し、2026-10-21までの検証を含む。以下はリポジトリルートで実行する。新しい出力名を使うこと。原本SVO2・bagは読み取りだけで、ドライバ、カメラ、RVizは起動しない。

## 環境

- 主処理：Python 3.12、NumPy 2.5.1、ROS 2 Jazzy。画像可視化：OpenCV 5.0.0。
- GUIコアの既存テスト：システム側NumPy 1.26.4 / SciPy 1.11.4。ユーザー側NumPy 2.5.1とシステム側SciPyの混在はABIエラーになる。この環境では `PYTHONNOUSERSITE=1` で解消した。システムのパッケージは変更していない。他PCは `calibration_gui/requirements.txt` を専用venvに入れる。
- このLinuxの `pyzed.sl` は `libnvidia-encode.so.1` 不足でimportできなかった。ZED再処理は未実行。保存済みのWindows出力を使った。
- `/data/experiment_data/scripts/` の2本のSDKスクリプトはリポジトリ版とSHA-256が一致したためコピー・変更は不要だった。

## 今回の実入力

| 種類 | パス |
|---|---|
| LiDAR | `/data/experiment_data/recordings/20261013_group001/recording_001/lidar` |
| ZED画像・深度 | `/data/experiment_data/recording_001_svo_review` |
| ZED BODY_34 | `/data/experiment_data/recording_001_zed_body_tracking` |
| 設定 | `config.json`（変更していない） |
| 相対外部変換 | `tracking/lidar_to_zed_left_eye.json`（部分校正値、変更していない） |
| 廊下座標 | `tracking/corridor_frame_livox_level.json`（変更していない） |

各処理のパス、設定値、実行時間は `final/tracks.metadata.json`、時刻監査は `recording_audit.json`、テストログは `final/` にある。追跡CSVは新しい列を要求せず従来の入出力を維持し、別ファイルで全フレームと来歴を追加した。CLIの意図的な変更は「時刻補正の明示」「上書き拒否」である。

## 追跡と真値なしレビュー

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/time -v python3 scripts/track_people.py \
  /data/experiment_data/recordings/20261013_group001/recording_001/lidar \
  --config config.json --output reports/reproduce_new/tracks.csv
python3 scripts/evaluate_tracking.py \
  --tracks reports/reproduce_new/tracks.csv \
  --frames reports/reproduce_new/tracks.frames.csv \
  --output-dir reports/reproduce_new/evaluation
```

`tracks.frames.csv`は検出ゼロのフレームも含む。これを分母に使わずにCSV行数だけで追跡率を出してはいけない。さらに人物が存在した時間の正解がなければ、全フレームに対する検出フレーム比は追跡成功率ではない。

比較用の変更前プログラムは `baseline/track_people.py` と `baseline/fuse_zed_lidar.py` に保存した。変更前の全bag処理を再現する場合、ROSのPYTHONPATHを消さずに次のように補う。

```bash
source /opt/ros/jazzy/setup.bash
PYTHONPATH="$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}" python3 \
  reports/review_20261020/baseline/track_people.py \
  /data/experiment_data/recordings/20261013_group001/recording_001/lidar \
  --config "$PWD/config.json" --output reports/reproduce_baseline_new/tracks.csv
```

基準コードの追跡CSVには全フレーム表がない。今回の同じbag・stride=1比較には改良版の全フレーム表を使用した。任意の別録画・別strideに流用しないこと。

## 合成軌跡による同一条件比較

```bash
python3 scripts/benchmark_tracking.py \
  --baseline-module reports/review_20261020/baseline/track_people.py \
  --output-dir reports/synthetic_reproduce_new
```

7ケース、各41時刻。検出器の後に既知の検出を注入し、確認条件は両版とも `min_hits=1`、`min_confirmed_consecutive_hits=2`、`min_track_displacement=0`、速度更新係数0.65。検出精度や人体の実測雑音は検証していない。近接する2人の別々の検出が得られる前提であり、LiDARクラスタが1個に融合した状況の解決ではない。欠測中の真の人物は正解に残し、予測だけのトラックを検出成功に数えない。

## 融合と時刻感度

```bash
python3 scripts/fuse_zed_lidar.py \
  --body-dir /data/experiment_data/recording_001_zed_body_tracking \
  --tracks reports/reproduce_new/tracks.csv --config config.json \
  --timestamp-offset-ms 0 \
  --sync-note "20261013同一epoch時計仮説。独立な同期計測なしの探索比較" \
  --output-dir reports/reproduce_new/fusion
```

今回は0、−100、+100 msで探索比較した。符号は `t_lidar_header = t_zed_image + offset`。残差が小さくなる補正値を採用したわけではない。相対位置の系統差・身体代表点の差が時間差と混同されるため、残差最小化だけでは同期の真値にならない。速度や側寄せの時間を評価するには、共通の急峻なイベントを複数回記録し、`t_lidar = a*(t_zed-t0) + b + t0`のoffsetとdriftを独立に評価する。今回driftは未推定。

融合HTMLのほか、`fused_trajectory.csv`にはZEDのみ・LiDARのみ・融合位置の列が残る。現在の融合処理はZED観測を起点とするため、ZED観測行が全くない時刻のLiDARだけの観測は新規行にならない。完全な非同期融合トラッカーではない。

## 保存点群・ZED投影の確認

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/render_sensor_review.py \
  --bag /data/experiment_data/recordings/20261013_group001/recording_001/lidar \
  --review-dir /data/experiment_data/recording_001_svo_review \
  --tracks reports/reproduce_new/tracks.csv --config config.json \
  --timestamp-offset-ms 0 --sync-note "同一時計を仮定した探索投影、同期精度は未確認" \
  --times-s 17 23 27 33 --output-dir reports/reproduce_new/sensor_review
```

画像の点はLiDARの全点、下段は水平化・廊下変換した点群。確定検出と予測を区別する。点の描画順は遠方→近傍だが、LiDARが見ていないカメラ遮蔽物の推論はしていない。`manifest.json`の画像時刻と点群時刻差を確認する。HTMLとJPEGを一緒に保管する。

旧 `compare_zed_lidar.py` は単独歩行の前景比較用。利用するなら `--lidar-time-offset-s`（**経過時刻差**）と対象IDを明示する。今回の追跡CSVの先頭headerを起点にすると `1.691819674 s` であり、bag起点の旧値を固定で使わない。

## 独立参照ラベルと評価分割

CSVの例（数値は形式例であり測定値ではない）:

```csv
timestamp_ns,person_id,x_m,y_m,vx_mps,vy_mps
1791863426622916075,participant_A,4.0,-0.2,,
1791863426622916075,participant_B,3.0,0.3,,
```

- 時刻は評価するLiDAR header時計に合わせる。座標は同じ設置の廊下座標[m]。各時刻の対象者を**全員**記入する。空の場面は `timestamp_ns` だけと空の `person_id` で表す。未ラベルの時刻は評価対象外。
- 床の測量済み格子と第三の上方カメラ等で、身体の床面基準点を定義する。LiDAR表面中央値と骨盤投影は別の点なので、参照の定義を先に統一する。クリックだけで画像からメートル位置が分かるとは仮定しない。
- 画像だけの人物ID・遮蔽ラベルは、2人を識別して注釈し、別の注釈者に確認してもらう。距離真値のないラベルから位置RMSEを作らない。この評価器のmetric CSV入力には独立なメートル参照が必要。
- 既知の床位置での静止、通常歩行、左右追越し、接近遮蔽、進路変更、8–12 mを含める。1人の往復だけで追越し性能を代表させない。
- 被験者・日・再設置単位で開発/検証/最終評価を分け、分割表を解析前に保存する。閾値調整した録画を最終評価に再利用しない。今回の合成ケースは回帰試験であり、独立した実験評価セットではない。
- 参照計測の誤差、注釈者間差、同期誤差も保存する。速度は独立参照から取得した場合だけ埋める。

```bash
python3 scripts/evaluate_tracking.py \
  --tracks reports/reproduce_new/tracks.csv \
  --frames reports/reproduce_new/tracks.frames.csv \
  --labels /path/to/independent_reference.csv \
  --reference-note "参照計測法、基準点、校正版、test分割名、注釈者版" \
  --gate-m 0.75 --time-tolerance-s 0.049 --id-switch-max-gap-s 1.0 \
  --output-dir reports/reference_evaluation_new
```

最大対応数→距離最小の一対一対応。確定かつ実検出だけを採点し、予測・未確定は除外。MAE/RMSEは対応できた標本の2D距離誤差なので、欠測率・偽陽性数・ゲートを必ず併記する。ID Switchは同じ参照IDに対するトラックID変化で、既定1秒以内の再対応を数える。長時間の断片化とは別。出力は標本単位のrecallであり、時間重み付き追跡成功率ではない。ラベル間隔が細かすぎて同じ点群を複数回採点しようとすると停止する。

回避開始時刻は現在自動判定しない。`kinematics.csv`の横位置と微分を確認し、事前に決めた基準区間・変位閾値・持続時間と、その感度を記録して評価する。検出開始位置からの横変位を「回避量」と自動同一視しない。

## 役割とIDの分離

新しい注釈JSONの形式:

```json
{
  "recording": "/data/experiment_data/recordings/YYYYMMDD_group001/recording_NNN",
  "actual_behavior_verified": true,
  "track_participants": {"12": "subject_1", "28": "subject_2"}
}
```

実際の映像で対応と実施内容を確認してから作る。`--roles annotation.json --recording-json .../recording.json --run-order run_order.json`を評価コマンドに追加する。役割・条件は注釈JSONに再記載せず、録画の計画ハッシュに一致する`run_order.json`から取得する。10月13日録画は計画ラベルが「追越し」でも実際は1人往復なので、今回は役割を割り当てなかった。IDが途中で変わった場合はその対応の根拠も別の注釈記録に残す。

## 校正の再利用・更新

1. **相対外部変換**：剛体固定が保たれれば再利用候補。左レンズ中心とLivox測距原点をCADで確認し、固定が動いた場合は再校正。既存JSONは床・壁の部分校正であり完全な6自由度校正ではない。
2. **設置校正**：IMU静止平均、床高、左右壁、廊下方向を設置ごとに取り直す。測量した廊下方向ランドマークで原点を固定しないと別設置の位置を直接比較できない。
3. `audit_recordings.py`で設置差と時刻連続性を点検する。IMU平均だけでは床高・yaw・原点を決められない。

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/audit_recordings.py --recordings /data/experiment_data/recordings \
  --config config.json --output reports/recording_audit_new.json
python3 scripts/estimate_corridor_frame.py \
  /data/experiment_data/recordings/20261013_group001/recording_001/lidar \
  --config config.json --output tracking/corridor_candidate_new.json
```

10月19日のArUco録画の画像/深度/IMU書き出しは `/data/experiment_data/aruco_20261019_review_new` に完了している。現在の書き出しでは校正板のLiDAR面が確定せず、offset `0.0` の探索でも有効平面が0件だった。`verified_lidar_minus_zed_seconds` が未設定なら元のコマンドは空文字を渡して失敗する。開始時刻の差をoffsetとして流用しないこと。

観察・原因・追加取得条件・SDK側の残作業は [2026-10-19 ArUco校正のタスク](ARUCO_20261019_NEXT_TASKS.md) に記録した。今回の校正データから候補は作られておらず、有効な外部校正JSONは変更していない。

新しい校正録画と独立に測ったoffsetが用意できた後のコマンド形:

```bash
read -r -p 'Verified offset seconds (LiDAR = ZED + offset): ' verified_lidar_minus_zed_seconds
: "${verified_lidar_minus_zed_seconds:?offsetを設定してください}"
[[ "$verified_lidar_minus_zed_seconds" =~ ^[+-]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][+-]?[0-9]+)?$ ]] || { echo 'offsetは数値で入力してください' >&2; exit 2; }
python3 scripts/calibrate_zed_lidar_aruco.py \
  --review-dir /data/experiment_data/<新しい校正レビュー出力> \
  --bag /data/experiment_data/recordings/<新しい校正録画>/lidar \
  --output-dir reports/aruco_calibration_<新規名> \
  --lidar-time-offset-seconds "$verified_lidar_minus_zed_seconds"
```

候補の適用は今回していない。既知サイズ板の面内広がり、点数、残差、外れ値マスク、条件数、leave-one-out並進変化量を確認し、別姿勢の板で再投影・面残差を検証する。条件付き並進共分散は法線・回転が固定という仮定であり、全6自由度の精度保証ではない。板を1–3 m等の近距離で複数方向に傾け、床・壁から離して撮る。現行の250×180 mm板が十分かは実際の点数・2方向の広がりで判定する。不足なら板を大きくするか静止積算時間を増やす。印刷模様をLiDARが認識する処理は使わない。

## 回帰確認

README既存の手順に追加の計測テストを含めた。

```bash
python3 -m compileall -q scripts tests
python3 -m unittest discover -s tests -v
PYTHONNOUSERSITE=1 python3 -m pytest calibration_gui/tests -q
```

保存bagを読むROSライブラリは使用したが、センサー・ZED SDK・RViz実機起動はしていない。HTMLはChrome headlessで描画確認し、投影JPEGも目視確認した。実機での録画開始停止、ZED再書き出し、GUIウィンドウ操作、RVizは今回の確認範囲外。
