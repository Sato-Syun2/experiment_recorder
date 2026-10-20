# ZED + Livox Experiment Recorder

歩行実験で、ZED 2i のSVO2とLivox Mid-360Sのrosbag2を試行単位で開始・終了するランチャーです。Livoxが未接続、ROS 2が利用不可、またはrosbag2の開始に失敗した場合も、警告を表示してZED録画だけを継続します。

## 必要環境

- Ubuntu / Python 3（標準ライブラリのみ）
- ZED SDK 5.4.0（または互換版）
- `ZED_Explorer` が `PATH` に存在すること（`which ZED_Explorer` で確認）
- 接続された ZED カメラ（ZED 2i など）
- ROS 2 Jazzy（Livox録画を使用する場合）
- RViz2（LiDAR点群を画面表示する場合）
- ビルド済みの `livox_ros_driver2`（既定では `/home/hp-ubuntu/ros2_ws`）
- Livox Mid-360S（未接続でもZED録画は可能）

ZEDは必須です。ZED SDK/カメラが利用できない場合は録画を中止します。Livoxは任意扱いで、利用できない場合はZEDのみのモードになります。

## 実行方法

プロジェクトルートから次を実行します。

```bash
python3 scripts/record_zed.py
```

既定の撮影・センサー設定はプロジェクトルートの `config.json` にあります。試行順・条件ID・役割・教示は、指定Excelを照合して生成する `run_order.json` が唯一の参照元です。

Excelを更新した場合は、録画を始める前に次を実行してください。表間に不一致があると、具体的な箇所を表示して既存の `run_order.json` を更新しません。

```bash
python3 scripts/build_run_order.py
```

`reports/run_order_audit.md` に18グループのセット順と、A/B/C1/C2の配布用18試行を被験者1・2の教示付きで一覧表示します。録画起動時は、生成元ExcelのSHA-256と計画データのSHA-256を再確認します。

```json
{
  "trial": 1,
  "group_number": 1,
  "output_dir": "/data/experiment_data/recordings",
  "compression_mode": 1,
  "resolution": "HD720",
  "fps": 30,
  "zed_explorer": "ZED_Explorer",
  "livox": {
    "enabled": true,
    "output_dir": "/data/experiment_data/recordings",
    "topics": ["/livox/lidar", "/livox/imu"],
    "detection_topic": "/livox/lidar",
    "connection_timeout_seconds": 5.0,
    "sensor_ip": "192.168.1.197",
    "ping_timeout_seconds": 1.0,
    "ros2_executable": "ros2",
    "setup_files": [
      "/opt/ros/jazzy/setup.bash",
      "/home/hp-ubuntu/ros2_ws/install/setup.bash"
    ],
    "start_driver": true,
    "driver_package": "livox_ros_driver2",
    "driver_launch_file": "rviz_MID360s_launch.py",
    "storage": "mcap"
  },
  "replay": {
    "point_cloud_topic": "/livox/lidar",
    "point_cloud_type": "auto",
    "converted_point_cloud_topic": "/livox/points",
    "fixed_frame": "livox_frame",
    "rviz_enabled": true,
    "bag_rate": 1.0,
    "bag_delay_seconds": 0.0,
    "loop": false
  }
}
```

録画データは `output_dir` 以下に保存します。`group_number` は開始グループ、`trial` は開始時に選ぶ全体試行番号です。パスを相対指定した場合は `config.json` のあるディレクトリが基準になります。`livox.output_dir` は旧形式のrosbag2を再生するときの検索先で、新しい録画はZEDと同じ録画フォルダへ保存します。`livox.enabled` を `false` にするとLivox処理を明示的に無効化できます。

計画データでは条件ID E01〜E18、通路幅、回避、前方者速度、追越方向、参加者役割、配布プリントの教示を1行にまとめています。録画前画面と `recording.json` は同じ `run_order.json` の試行行を使います。

コマンドラインオプションで設定ファイルの値を一時的に上書きすることもできます。

```bash
python3 scripts/record_zed.py --trial 1
python3 scripts/record_zed.py --group-number 2
python3 scripts/record_zed.py --output-dir /data/experiment_data/recordings
python3 scripts/record_zed.py --config /path/to/my_config.json
python3 scripts/record_zed.py --run-order /path/to/run_order.json
```

## 録画の開始・終了

1. 表示されたグループ、試行番号、通路幅、Take番号、被験者1・2の役割と配布プリントの教示を確認します。`←` / `→`でG01〜G18、`↑`で試行番号を1つ戻し、`↓`で1つ進めます。Enterで録画開始、`Q`で終了します。セット切替、通路幅変更、役割変更がある試行では確認枠を表示します。
2. ランチャーが `sensor_ip` へpingした後、`/livox/lidar` を確認します。pingは補助的な確認で、ICMP応答がなくてもROS 2トピック検出とドライバ起動を続けます。トピックがまだない場合はMid-360Sドライバを起動して既定で5秒待ちます。
3. Livox接続時は `/livox/lidar` と `/livox/imu` のrosbag2録画を開始します。
4. Livox未接続時は大きな警告を表示し、rosbag2をスキップしてZEDだけを録画します。
5. `ZED_Explorer -o <file>.svo2 --cpm 1 --resolution HD720 --frequency 30` を起動します。
6. 録画中にスペース、`S`、またはEnterを押すと、すべての録画プロセスへSIGINTを送り、正常終了と書き込み完了を待ちます。
7. 保存検証後は進捗とTake番号を `output_dir/.experiment_progress/` に保存します。再録失敗は必要な試行として表示し、次回起動時に優先します。進捗ファイルはグループと計画SHA-256ごとに分けて保存するため、計画変更後の進捗とは混ざりません。

基本54試行の完了後は停止し、第4セットを実施するか確認します。Yを選んでも録画は始まらず、55試行目の内容が表示された後にEnterで開始します。第4セットはグループ順表にある未実施側のC1/C2です。

録画中の画面にもグループ、試行番号、通路幅、Take番号、役割と教示を表示します。経過時間と停止キーも表示し、`Ctrl+C`は緊急終了操作として使用できます。

録画中は経過時間を表示します。録画データは次の形式で保存します。

```text
/data/experiment_data/recordings/
└── <YYYYMMDD>_group<グループ番号>/
    └── recording_<録画ID>/
        ├── recording_<録画ID>.svo2
        ├── recording.json
        └── lidar/
            ├── metadata.yaml
            └── *.mcap
```

日付は録画開始日です。録画IDは日付とグループごとに `recording_001` から順に割り当て、既存データを上書きしません。SVO2とLiDAR rosbag2を同じ録画フォルダにまとめています。`recording.json` には録画ID、グループ、全体試行番号、セット、用紙内番号、基本／追加、Take番号、条件IDと要因、被験者1・2の役割・原文教示、計画版・計画ハッシュ・元Excelのファイル名とSHA-256を記録します。条件や追加試行で並べ替えられるラベルを含みます。

録画IDフォルダにはSVO2とLiDARのデータが入ります。Livox未接続時はSVO2だけが作成されます。

## Livox未接続時

接続トピックが見つからない場合は、次のような警告を表示します。

RVizが起動しているだけでは接続済みと判定せず、`/livox/lidar` に実際のPublisherがある場合だけ点群録画を開始します。

```text
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 【警告】Livox未接続
 この試行では点群を保存せず、ZEDのみ録画します。
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
```

この状態でもZEDの録画開始・停止・SVO2検証は通常どおり行います。Livoxの失敗を理由にZEDの成果物を削除したり、ZED録画を途中終了したりしません。

既定ではセンサーIPを `192.168.1.197` として確認し、`ros2 launch livox_ros_driver2 rviz_MID360s_launch.py` を使用します。このlaunchはPointCloud2形式（`/livox/lidar`）のドライバとRViz2を起動します。センサーIPやPC側IPは、`config.json` とLivoxドライバ側の `MID360s_config.json` を実機ネットワークに合わせて設定してください。pingは補助確認のため、pingに応答しない構成でもROS 2接続を試行します。事前ping自体を無効化する場合は `sensor_ip` を空文字列にします。

## 録画設定（CPM / 解像度 / FPS）

`config.json` の既定値は次のとおりです。

- `compression_mode: 1`（`--cpm 1`、ZED SDKのLossless ZST）
- `resolution: "HD720"`（`--resolution HD720`）
- `fps: 30`（`--frequency 30`）

以前の環境ではデフォルト圧縮設定で録画に失敗したため `--cpm 0` を使用していましたが、現在の設定ではユーザー指定どおり `--cpm 1` を使用します。問題が発生した場合は、設定ファイルの `compression_mode` を変更して切り分けてください。

## Ctrl+C

PythonランチャーにCtrl+Cが送られた場合、録画中のrosbag2と`ZED_Explorer`へSIGINTを送り、両方が正常終了するまで待ってからランチャーを終了します。SVO2やrosbag2の破損を避けるため、SIGTERM / SIGKILLによる強制終了は行いません。

## 検証内容

ZEDは終了コード、ファイル存在、非ゼロサイズを確認します。Livoxはrosbag2の終了コード、`metadata.yaml`、非ゼロサイズの`.mcap`または`.db3`を確認します。Livoxドライバとrosbag2のログは `logs/` に保存します。

## SVOとLiDAR点群の同時再生

録画済みのSVO2と、同じ録画フォルダに保存したrosbag2を1つのランチャーから同時に再生できます。SVO2は`ZED_Explorer`、LiDARの点群は`ros2 bag play`で配信し、既定では`RViz2`も起動して表示します。新しい保存形式ではSVO2の隣にある`lidar/`を自動で使います。現在の録画設定（`rviz_MID360s_launch.py`）ではLivoxを`PointCloud2`形式で保存するため、再生時の変換は不要です。`msg_MID360s_launch.py`で保存した`CustomMsg`形式のbagも、再生時に自動で`PointCloud2`へ変換できます。

```bash
python3 scripts/replay_zed_lidar.py \
  '/data/experiment_data/recordings/<日付>_group001/recording_001/recording_001.svo2'
```

録画フォルダ内の`lidar/`を自動検出するため、新しい保存形式では`--bag`を省略できます。旧形式や別の場所にあるbagを再生するときは`--bag`を指定します。

```bash
python3 scripts/replay_zed_lidar.py \
  '/data/experiment_data/recordings/<日付>_group001/recording_001/recording_001.svo2'
```

再生中にEnterまたはCtrl+Cを押すと、ZED、rosbag2、RViz2を停止します。RViz2を使わず点群トピックだけ配信する場合は`--no-rviz`を指定します。点群トピックやフレーム名が環境と異なる場合は、次のように上書きできます。

再生開始時はrosbag2とZED_Explorerを連続して起動し、片方の再生終了を検知したらもう片方も停止します。ZED_ExplorerはSVOの終端で先頭へ戻ることがあるため、rosbag2終了の監視を短い間隔で行い、終端後の短いループ表示を抑制しています。なお、録画側の開始・終了処理やファイルごとの実時間差がある場合は、`--bag-delay 0.1`のようにLiDAR側の開始遅延を微調整できます。

```bash
python3 scripts/replay_zed_lidar.py \
  '/data/experiment_data/recordings/<日付>_group001/recording_001/recording_001.svo2' \
  --bag '/data/experiment_data/recordings/<日付>_group001/recording_001/lidar' \
  --topic /livox/lidar --fixed-frame livox_frame
```

`--fixed-frame`はPointCloud2メッセージの`header.frame_id`と一致させてください。点群型の自動判定が使えない場合は`--point-cloud-type custom`または`--point-cloud-type pointcloud2`を指定できます。SVO2とrosbag2の記録タイムスタンプが読める場合は開始時刻の差を自動補正しますが、フレーム単位のハード同期ではありません。残る差を手動調整する場合は`--bag-delay 0.1`のように指定できます。再生速度は`--rate 0.5`、ループは`--loop`で指定できます。

## 点群から人物位置を追跡

録画済みのrosbag2を直接読み込み、点群の前処理、人物候補クラスタの抽出、フレーム間のID追跡を行えます。人数は2人に固定していません。標準出力は1行1トラックのCSVで、`x,y,z`が元の`livox_frame`座標、`track_id`が追跡ID、`vx,vy`が速度です。重力水平化を有効にした場合は`level_x/y/z`と`level_vx/vy/vz`も出力し、`level_speed_xy`が水平面内の速度です。校正後は`corridor_x,y,z`と`corridor_vx,vy,vz`に、廊下方向・廊下幅方向・高さの座標と速度も出力します。

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/track_people.py \
  /data/experiment_data/recordings/20261005_group001/recording_003/lidar \
  --topic /livox/lidar
```

### 廊下中心座標の校正

左右の壁を点群から推定し、壁間の中心線を原点とする座標系を作成します。最初の30フレームを使う例です。

```bash
source /opt/ros/jazzy/setup.bash
source /home/hp-ubuntu/ros2_ws/install/setup.bash
python3 scripts/estimate_corridor_frame.py \
  /data/experiment_data/recordings/20261005_group001/recording_001/lidar
```

校正ファイルは`config.json`の`tracking.corridor_frame_file`に保存されます。重力水平化が有効な場合は、校正スクリプトも点群を水平化してから壁を推定します。`tracking.output_coordinate_frame`が`corridor`の場合、以後の追跡CSVへ廊下座標が自動的に追加されます。`corridor_x`は廊下方向（録画時の+X側を正）、`corridor_y`は左右壁の中間から高い側の壁方向を正、`corridor_z`は高さです。別の廊下やセンサー設置位置で使う場合は、その環境の録画から校正をやり直してください。

### ZED–LiDAR間の床・壁校正

`tracking/lidar_to_zed_left_eye.json`はLivox `livox_frame`からZED左目の`IMAGE`座標（x右、y下、z前方）への変換です。`point_zed = R × point_livox + translation`、位置・並進の単位はmです。ZED SDKの既定単位mmを直接比較するときはmへ変換してください。

今回の`20261013_group001/recording_001`では、人物が入る前の廊下8.5秒からLiDAR 85フレームとZED深度17フレームを使い、床と左右壁の平面を合わせました。回転は初期値から5.44°補正し、校正後の並進は`[-0.0221, -0.1270, 0.0071] m`です。両側壁の法線、床高、横位置を反映しています。

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/calibrate_zed_lidar_planes.py \
  --review-dir /data/experiment_data/recording_001_svo_review \
  --bag /data/experiment_data/recordings/20261013_group001/recording_001/lidar \
  --output-dir tracking/20261013_group001_recording_001_plane_calibration
```

校正プログラムは共有済みのZEDレビュー深度とLiDAR rosbagを読むため、今回のデータについて別PCでの追加解析は不要です。既定では候補だけを別フォルダーへ出力します。確認した候補を有効化するときは、新しい空の出力先を指定して`--apply`を付けます。結果には面ごとの点数・残差・法線角度と、対応する校正JSONが保存されます。

床と平行な側壁だけでは廊下方向の並進は決まりません。現在のJSONはその1自由度を初期取り付け測定値から保持しています。またLiDAR壁間隔は2.387 m、ZED深度からの壁間隔は2.571 mで、約0.184 mの差があります。ZED深度の面形状や平面対応に由来する残差があるため、cm単位の位置精度を保証する値ではありません。完全な6自由度校正が必要な場合は、両センサーに共通して写る廊下端の横断壁、または既知位置の平面ターゲットを追加記録してください。そのSVO深度は既存の共有スクリプト`/data/experiment_data/scripts/export_svo_review.py`で書き出せます。

### ArUcoパネルを使ったZED–LiDAR相対校正

250 × 180 mmの板の中央に、150 mm角のArUco（ID 1、辞書`DICT_4X4_50`）を貼ります。1本の録画の中で板を動かし、**各位置・向きで約1秒止めて**、これを10姿勢ほど繰り返してください。動かし続けず、位置だけでなく傾きも上下・左右に変えます。両センサーから板が見えるようにしてください。カメラの絶対座標は不要です。

ZED SVO2と`/livox/lidar`・`/livox/imu` bagは実験時と同じ設定で同時録画し、校正専用フォルダーへ保存します。`record_zed.py`はグループ・試行を`run_order.json`へ紐付けて進捗も更新するため、校正用に番号を適当に選ばないでください。

ZED SDK対応PCで、SVOからIMU付きレビューを出力します。共有用スクリプトがなければ`export_svo_review.py`を`/data/experiment_data/scripts/`へコピーしてください。

```powershell
python "\\tsclient\_data_experiment_data\scripts\export_svo_review.py" `
  "C:\data\calibration.svo2" `
  --output "\\tsclient\_data_experiment_data\calibration_svo_review" `
  --every-seconds 0.1 --include-imu
```

Ubuntu側で、同じ録画のLiDAR bagを指定します。bagに`/livox/lidar`と`/livox/imu`が必要です。時刻差がある場合だけ`--lidar-time-offset-seconds`を調整してください（LiDAR時刻 = ZED時刻 + offset）。

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/calibrate_zed_lidar_aruco.py \
  --review-dir /data/experiment_data/calibration_svo_review \
  --bag /data/experiment_data/recordings/<校正録画>/lidar \
  --output-dir tracking/<校正録画>_aruco_calibration \
  --lidar-time-offset-seconds 0.0
```

重力方向でピッチ・ロールを補い、板の複数法線と面位置で残りの回転・並進を推定します。SVOから読んだZED IMU値を固定して使う場合は`--zed-gravity-acceleration-m-s2 X Y Z`（m/s²）を指定します。候補JSONと残差レポートを確認し、適用するときだけ`--apply`を追加してください。OpenCVの`cv2.aruco`が必要です。

結果は既定で `tracking/<bag名>_tracks.csv` に保存されます。現在のLivox録画は`PointCloud2`なので上記のまま動きます。`CustomMsg`形式のbagは次のように指定できます。

```bash
python3 scripts/track_people.py <bag-directory> \
  --topic /livox/lidar --message-type custom
```

リアルタイムのトピックを処理してRViz2へ人物の中心、歩行軌跡、速度矢印、座標と速度ラベルを出すこともできます。

```bash
python3 scripts/track_people.py --live \
  --topic /livox/lidar --publish-markers
```

録画済みのPointCloud2を点群と人物マーカー付きで順番にRViz2へ再生する場合は、次のスクリプトを使えます。各bagのCSVも同時に更新されます。

```bash
python3 scripts/replay_tracking_rviz.py \
  /data/experiment_data/recordings/20261005_group001/recording_001/lidar \
  /data/experiment_data/recordings/20261005_group001/recording_002/lidar \
  --topic /livox/lidar --rate 1.0
```

RVizには`/tracking/points`（点群）と`/people/tracks`（人物マーカー）を表示します。人物の中心にID別の色付き球を置き、直近60フレームの軌跡を線でつなぎます。中心から伸びる矢印は進行方向と速度を表します。人物の横には各フレームの中心座標`x,y,z`（m）、水平速度`vx,vy`（m/s）、合成速度（m/s）を表示します。`min_track_displacement`により、移動が確認できたトラックだけをRVizに表示します。各bagの再生が終わった後も最後の表示を確認できます。

解析済みCSVから動きを再生する場合は、次のコマンドでブラウザ用の単体HTMLを生成して開きます。平面軌跡、フレーム送り・再生操作、人物ごとの速度時系列を同じ時間軸で確認できます。追加のPythonライブラリは不要です。

```bash
python3 scripts/replay_tracks_csv.py tracking/recording_003_tracks.csv
```

HTMLはCSVと同じ場所に`*_replay.html`として保存されます。空のフレーム時刻は前後のCSV時刻から補間し、先頭部分など補間できない区間は既定10 FPSで補います。FPSを変更する場合は`--fps 20`、HTMLだけ生成してブラウザを開かない場合は`--no-open`を指定できます。
カメラ映像と一緒に見る場合は、同じ試行のSVO2を`--svo`で指定します。CSV・SVOの試行名が一致することを確認し、対応するLiDAR bagを自動推定して、記録タイムスタンプの開始差を補正してからHTMLと`ZED_Explorer`を起動します。bagの場所を明示する場合は`--sync-bag`を指定します。

```bash
python3 scripts/replay_tracks_csv.py \
  tracking/recording_003_tracks.csv \
  --svo /data/experiment_data/recordings/20261005_group001/recording_003/recording_003.svo2
```

ZED_Explorerの再生速度はアプリ側で操作します。`--rate`はHTMLグラフの速度に適用されるため、カメラ映像と合わせる場合は通常の1倍速を使ってください。HTMLは既定の`--coordinate-frame auto`で校正済みの廊下座標を優先します。生のLiDAR座標で表示する場合は`--coordinate-frame livox`、廊下座標を必須にする場合は`--coordinate-frame corridor`を指定します。ZED画像の左右へ軌跡を重ねるには、別途ZED-LiDAR間の外部パラメータが必要です。`--zed-explorer`で実行ファイルを上書きできます。


点群を確認したい場合は、カメラ映像の代わりに解析に使ったrosbagを`--bag`で指定すると、RVizに点群とCSVの人物位置・速度マーカーが表示されます。ブラウザの速度グラフも指定した再生速度で自動再生します。

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/replay_tracks_csv.py \
  tracking/test_003_20260917_145451_tracks.csv \
  --bag /data/experiment_data/recordings/20261005_group001/recording_003/lidar \
  --rate 1.0
```

CSVのマーカーは点群メッセージのタイムスタンプでbagと対応付けます。HTML側のシーク・一時停止操作はRVizの点群再生には連動しません。

追跡処理は`config.json`の`tracking`ブロックで調整します。主な設定は次のとおりです。

- `roi`：処理座標系での範囲。重力水平化が有効なら、センサー座標を水平化・床面移動した後の座標で指定します。
- `output_coordinate_frame` / `corridor_frame_file`：追跡CSVへ廊下中心座標を追加する設定。校正ファイルは`estimate_corridor_frame.py`で作成します。
- `gravity_alignment`：IMUから得た上方向へ点群を回転し、床面を`z=0`に合わせる設定です。センサー姿勢や床高が変わった場合は上方向・床高と廊下校正を更新します。水平化後のRVizマーカーは点群の元座標へ戻して重ねます。
- `ground_mode` / `ground_z`：床面の設定。重力水平化が有効な場合は、移動後の床が概ね`z=0`です。床の高さが未知なら`auto`を試してください。
- `background_mode`：`temporal`で初期フレームから壁・家具などの静的点群を除去します。背景が動く場所では`none`にします。
- `cluster_*`：人物候補の大きさ・密度。点が疎な場合は`min_cluster_points`を下げ、候補が分断される場合は`cluster_tolerance`を上げます。`far_detection_start`以遠では、遠距離の点密度低下を考慮して`far_min_cluster_points`、`far_min_cluster_width`、`far_min_cluster_depth`を使用します。
- `cluster_background_voxel_size` / `cluster_background_min_presence`：点単位の背景除去を通過した壁・設備の候補を、クラスタ中心の滞在率でも除去します。
- `max_match_distance` / `max_track_speed` / `max_missed_frames`：歩行速度や遮蔽に応じたID追跡設定です。
  `max_match_distance`は一時的な点群中心のずれを同じIDへ再接続する距離ゲートです。
- `min_track_displacement`：この距離以上の移動が確認できるまでRVizには表示しません。静止物の誤検出を抑える設定で、静止人物も追跡したい場合は`0`にできます。
- `min_motion_consistency`：連続検出区間の移動量と経路長の比率です。方向が頻繁に反転する静的クラスタを人物として確定しないために使います。
- `motion_consistency_window`：移動の一貫性を評価する連続フレーム数です。短いクラスタ揺らぎを確定しないため、既定値は15です。

`background_mode: temporal`では、既定で最初の30点群フレームを背景学習に使うため、その間は人物候補を出力しません。開始直後から追跡したい場合は`background_mode: none`にするか、`background_learning_frames`を短くしてください。床の高さやセンサーの向きが録画ごとに異なる場合は、`gravity_alignment`、`ground_z`、`roi`を点群座標系に合わせて変更します。

これは点群だけを使う汎用的な幾何ベースのベースラインです。人数の上限で候補を削るのではなく、点単位の背景除去、候補クラスタの空間的な滞在率、移動量による確認を組み合わせています。壁や人が密着する、長時間遮蔽される、人物同士が交差する場合はIDの入れ替わりや見失いが起こり得るため、CSVの`status`、`confidence`、`detected`も併せて確認してください。より高い精度が必要になった場合は、この出力を教師データとして、人物検出モデルやセンサー間の座標変換を追加できます。

## ZED SDK骨格解析（別PC）

`scripts/analyze_svo_body_tracking.py`はZED SDK対応PCでSVO2を全フレーム処理し、BODY_34の2D/3Dキーポイント、人物ID、SDKの位置・速度、関節姿勢、人物領域の深度要約を書き出します。SDKによる人物追跡と深度推定を使い、骨格オーバーレイ画像とコンタクトシートも作成します。ZED SDK 5.4、対応する`pyzed`、CUDA、NumPy、OpenCVが必要です。

スクリプトを `/data/experiment_data/scripts/` に置き、Windows側では共有パスを使います。SVO2は外部PCのローカルパスから読み、出力は録画原本と分けた新しいフォルダーに保存してください。

```powershell
python "\\tsclient\_data_experiment_data\scripts\analyze_svo_body_tracking.py" `
  "C:\data\recording_001.svo2" `
  --output "\\tsclient\_data_experiment_data\recording_001_zed_body_tracking"
```

出力フォルダーには、全フレーム時刻の `frames.csv`、人物ごとの `bodies.csv`、SDK・カメラ情報の `camera.json`、確認画像の `preview_frames/` と `contact_sheet.jpg` が作られます。CSVのキーポイント配列は `camera.json` の `keypoint_names` 順です。人物領域の深度統計に加え、骨盤画像位置の近傍から取った深度値と逆投影座標も保存します。座標はm単位のZED `IMAGE` 座標（右・下・前）です。

## ZED・LiDAR軌道統合

ZED側の出力を共有フォルダーからこのPCへ見せた後、`scripts/fuse_zed_lidar.py`でLiDAR追跡CSVと統合します。各ZED画像時刻とLiDARメッセージ時刻のナノ秒値を直接照合し、確認済み・検出済みLiDARトラックだけを対応付けます。短い連続区間内でのみLiDAR位置を補間し、廊下校正とZED外部変換を使って座標をそろえます。

```bash
python3 scripts/fuse_zed_lidar.py \
  --body-dir /data/experiment_data/recording_001_zed_body_tracking \
  --tracks tracking/20261013_group001_recording_001_plane_calibrated_tracks.csv \
  --config config.json \
  --output-dir tracking/20261013_group001_recording_001_zed_lidar_fusion_plane_calibrated
```

既定の時刻補正は0 msです。今回の録画ではZED先頭画像時刻とrosbag開始時刻の差が約1.675882秒となることを確認できていますが、統合では固定値でなく各サンプルの記録タイムスタンプを使います。時計系が異なる別録画で補正が必要な場合だけ `--timestamp-offset-ms` を明示します。

統合結果は `fused_trajectory.csv`、`summary.json`、`associations.json`、`fused_trajectory.html` に出力されます。CSVにはセンサー別位置、SDK深度を使った骨盤位置、信頼度に応じた統合位置、ZED SDK速度と統合速度、同期差、深度要約、BODY_34から求めた左右の膝・足首・股関節と体幹の角度を含みます。センサー間の位置が合う場合、廊下方向の統合位置にはLiDARを最低0.70の比率で反映し、左右位置・高さは各センサーの品質に応じて統合します。比率は解析用の初期ヒューリスティックで、正解位置による校正値ではありません。SDKの骨格関節は推定点なので、角度は解剖学的な校正値ではありません。位置の不連続が大きいフレームは速度算出から除外し、センサー間の位置差が許容値を超えたフレームでは統合値を作りません。

`summary.json`の位置残差はLiDAR人物クラスタ中心とZEDの骨盤位置推定値の差です。真値に対する測位誤差ではありません。

統合後の廊下軌跡・速度・骨格角度を図と数値要約にまとめるには、次を実行します。出力先には新しいフォルダーを指定してください。入力CSVや録画原本は変更しません。

```bash
python3 scripts/plot_fused_trajectory.py \
  --csv tracking/20261013_group001_recording_001_zed_lidar_fusion_plane_calibrated/fused_trajectory.csv \
  --corridor-frame tracking/corridor_frame_livox_level.json \
  --output-dir reports/20261013_group001_recording_001_gait_assessment
```

`index.html`に廊下平面軌跡、廊下座標の位置時系列、水平速度と廊下方向速度、BODY_34の膝角度・体幹傾きをまとめます。SVG図と`metrics.json`も保存します。速度図にはフレームごとの速度と連続区間内の中央1秒中央値を併記し、速度参考線は健康成人研究の集団平均を示す目安として扱います。SDK骨格と位置推定の限界があるため、これだけで歩容の自然さや臨床的な正常性は判定できません。

## 統合位置と関節のRViz表示

LiDAR点群と統合人物位置・ZED骨格を同じRViz画面に表示するには、次のスクリプトを使います。統合CSVの時刻に最も近いZED観測を各LiDAR点群フレームへ対応付け、廊下座標で記録された人物位置と関節を点群の元座標フレームへ戻して配信します。既定では時刻差80 ms以内の観測を表示し、信頼度不足の関節と連続性判定で除外された人物位置は表示しません。

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/replay_fused_trajectory_rviz.py \
  /data/experiment_data/recordings/20261013_group001/recording_001/lidar \
  --fused-csv tracking/20261013_group001_recording_001_zed_lidar_fusion_plane_calibrated/fused_trajectory.csv \
  --config config.json --rate 1.0
```

RVizには白いLiDAR点群、人物の色付き中心球・短い軌跡・座標ラベル、BODY_34の関節点と骨格線を表示します。ラベルは統合状態と速度を示します。共有タイムスタンプで位置合わせするため、時刻補正が必要な別録画に限り `--timestamp-offset-ms` を指定してください。最後まで再生した時点でRVizを閉じる場合は `--exit-after-playback` を付けます。

今回のLiDAR→ZED変換は床・平行壁で回転と横位置・高さを合わせた部分校正値です。廊下方向の並進は初期取り付け値から保持しています。追跡CSVを校正後に作り直してから融合・RViz再生を行ってください。ZED/LiDAR投影の再確認ページは`tracking/20261013_group001_recording_001_zed_lidar_comparison_plane_calibrated.html`です。解析スクリプトは元SVO2・rosbagを変更せず、出力先が空でない場合は上書きしません。

## コード構成と回帰確認

| ファイル | 主な役割 |
| --- | --- |
| `scripts/record_zed.py` | 設定読み込み、Livox接続確認、ZEDとrosbag2の録画・保存確認 |
| `scripts/replay_zed_lidar.py` | ZED・rosbag2・RViz2の同時再生と子プロセス管理 |
| `scripts/playback_timeline.py` | SVO2とrosbag2の記録時刻を読み、再生開始差を計算 |
| `scripts/export_svo_review.py` | ZED SDKでSVO2から時刻付き画像、任意の深度・画像同期IMUを書き出し |
| `scripts/track_people.py` | 点群の前処理、人物候補の検出、ID追跡、CSV・マーカー出力 |
| `scripts/analyze_svo_body_tracking.py` | 別PCのZED SDKでSVO2全フレームの骨格・人物ID・深度を出力 |
| `scripts/calibrate_zed_lidar_planes.py` | ZED深度とLiDARから床・平行壁面を合わせ、観測できる外部変換成分を校正 |
| `scripts/calibrate_zed_lidar_aruco.py` | 複数姿勢のArUcoパネル面を対応付け、ZED–LiDAR間の6自由度変換候補を作成 |
| `scripts/fuse_zed_lidar.py` | ZED骨格・深度と確定LiDARトラックを時刻・廊下座標で統合 |
| `scripts/plot_fused_trajectory.py` | 統合CSVから軌跡・位置・速度・姿勢のSVG図と評価レポートを生成 |
| `scripts/replay_fused_trajectory_rviz.py` | rosbag点群に統合人物位置とZED BODY_34骨格を重ねてRViz再生 |
| `scripts/corridor_frame.py` / `scripts/estimate_corridor_frame.py` | 廊下座標の推定と校正ファイルの保存 |
| `scripts/replay_tracks_csv.py` / `scripts/tracking_replay.html` | 追跡CSVの整理と単体HTMLの生成・表示 |
| `scripts/replay_tracking_rviz.py` / `scripts/convert_livox_custom.py` | RVizでの追跡再生とCustomMsg点群の変換 |

実機を使わずに主要な保存名、記録時刻、CSV補間、ID維持、座標変換を確認できます。追跡関連の確認にはNumPyが必要です。

```bash
python3 -m compileall -q scripts tests
python3 -m unittest discover -s tests -v
```

ZEDカメラ、Livox、ROS 2、RViz2を起動する一連の操作は実機環境で確認してください。
