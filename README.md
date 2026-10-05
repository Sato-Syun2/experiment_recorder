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

既定の設定はプロジェクトルートの `config.json` にあります。保存先や保存名を変更するときは、このファイルを編集してから実行してください。

```json
{
  "condition": "test",
  "trial": 1,
  "output_dir": "recordings/zed",
  "filename_template": "{condition}_{trial:03d}_{timestamp}.svo2",
  "compression_mode": 1,
  "resolution": "HD720",
  "fps": 30,
  "zed_explorer": "ZED_Explorer",
  "livox": {
    "enabled": true,
    "output_dir": "recordings/livox",
    "bag_name_template": "{condition}_{trial:03d}_{timestamp}",
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

`output_dir` の相対パスは `config.json` のあるディレクトリを基準にします。`filename_template` では次の置換項目を使用できます。

- `{condition}`：condition（ファイル名に安全な文字へ整形）
- `{trial:03d}`：3桁の試行番号
- `{timestamp}`：録画開始時刻（`YYYYMMDD_HHMMSS`）

`livox.output_dir` と `livox.bag_name_template` も同じ規則です。`livox.enabled` を `false` にするとLivox処理を明示的に無効化できます。

コマンドラインオプションで設定ファイルの値を一時的に上書きすることもできます。

```bash
python3 scripts/record_zed.py --condition walking --trial 1
python3 scripts/record_zed.py --output-dir /path/to/recordings
python3 scripts/record_zed.py --config /path/to/my_config.json
```

## 録画の開始・終了

1. 表示された試行情報（condition / trial / ZED出力ファイル名）を確認し、Enterを押します。画面の`状態`と`操作`を確認してください。
2. ランチャーが `sensor_ip` へpingした後、`/livox/lidar` を確認します。pingは補助的な確認で、ICMP応答がなくてもROS 2トピック検出とドライバ起動を続けます。トピックがまだない場合はMid-360Sドライバを起動して既定で5秒待ちます。
3. Livox接続時は `/livox/lidar` と `/livox/imu` のrosbag2録画を開始します。
4. Livox未接続時は大きな警告を表示し、rosbag2をスキップしてZEDだけを録画します。
5. `ZED_Explorer -o <file>.svo2 --cpm 1 --resolution HD720 --frequency 30` を起動します。
6. 録画中にスペース、`S`、またはEnterを押すと、すべての録画プロセスへSIGINTを送り、正常終了と書き込み完了を待ちます。
7. 保存完了後はEnterで次の試行（001, 002, 003, ...）へ進み、`q`で終了します。

録画中の画面には状態、経過時間、Livoxの録画状態、現在の保存先、操作キーが表示されます。`Ctrl+C`は緊急終了操作として引き続き使用できます。

録画中は経過時間を表示します。ファイル名は原則として次の形式です。

```text
<condition>_<trial>_<録画開始時刻>.svo2
```

例: `test_001_20260831_163000.svo2`

ZEDの保存先は `recordings/zed/`、Livoxの保存先は `recordings/livox/` です。既存ファイルやrosbagディレクトリは上書きせず、同名の場合は `_01` などの連番を付けます。

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

録画済みのSVO2と、同じ試行で保存したrosbag2を1つのランチャーから同時に再生できます。SVO2は`ZED_Explorer`、LiDARの点群は`ros2 bag play`で配信し、既定では`RViz2`も起動して表示します。現在の録画設定（`rviz_MID360s_launch.py`）ではLivoxを`PointCloud2`形式で保存するため、再生時の変換は不要です。`msg_MID360s_launch.py`で保存した`CustomMsg`形式のbagも、再生時に自動で`PointCloud2`へ変換できます。

```bash
python3 scripts/replay_zed_lidar.py \
  recordings/zed/test_001_20260907_102701.svo2 \
  --bag recordings/livox/test_001_20260907_102701
```

SVO2のファイル名とrosbag2のディレクトリ名が同じ場合は`--bag`を省略できます。

```bash
python3 scripts/replay_zed_lidar.py recordings/zed/<file>.svo2
```

再生中にEnterまたはCtrl+Cを押すと、ZED、rosbag2、RViz2を停止します。RViz2を使わず点群トピックだけ配信する場合は`--no-rviz`を指定します。点群トピックやフレーム名が環境と異なる場合は、次のように上書きできます。

再生開始時はrosbag2とZED_Explorerを連続して起動し、片方の再生終了を検知したらもう片方も停止します。ZED_ExplorerはSVOの終端で先頭へ戻ることがあるため、rosbag2終了の監視を短い間隔で行い、終端後の短いループ表示を抑制しています。なお、録画側の開始・終了処理やファイルごとの実時間差がある場合は、`--bag-delay 0.1`のようにLiDAR側の開始遅延を微調整できます。

```bash
python3 scripts/replay_zed_lidar.py recordings/zed/<file>.svo2 \
  --bag recordings/livox/<bag> \
  --topic /livox/lidar --fixed-frame livox_frame
```

`--fixed-frame`はPointCloud2メッセージの`header.frame_id`と一致させてください。点群型の自動判定が使えない場合は`--point-cloud-type custom`または`--point-cloud-type pointcloud2`を指定できます。SVO2とrosbag2の記録タイムスタンプが読める場合は開始時刻の差を自動補正しますが、フレーム単位のハード同期ではありません。残る差を手動調整する場合は`--bag-delay 0.1`のように指定できます。再生速度は`--rate 0.5`、ループは`--loop`で指定できます。

## 点群から人物位置を追跡

録画済みのrosbag2を直接読み込み、点群の前処理、人物候補クラスタの抽出、フレーム間のID追跡を行えます。人数は2人に固定していません。標準出力は1行1トラックのCSVで、`x,y,z`が`livox_frame`での人物位置、`track_id`が追跡ID、`vx,vy`が速度です。校正後は`corridor_x,y,z`と`corridor_vx,vy,vz`に、廊下方向・廊下幅方向・高さの座標と速度も出力します。

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/track_people.py \
  recordings/livox/test_003_20260917_115013 \
  --topic /livox/lidar
```

### 廊下中心座標の校正

左右の壁を点群から推定し、壁間の中心線を原点とする座標系を作成します。最初の30フレームを使う例です。

```bash
source /opt/ros/jazzy/setup.bash
source /home/hp-ubuntu/ros2_ws/install/setup.bash
python3 scripts/estimate_corridor_frame.py \
  recordings/livox/test_001_20260917_145114
```

校正ファイルは`tracking/corridor_frame.json`に保存されます。`config.json`の`tracking.output_coordinate_frame`が`corridor`の場合、以後の追跡CSVへ廊下座標が自動的に追加されます。`corridor_x`は廊下方向（録画時の+X側を正）、`corridor_y`は左右壁の中間から高い側の壁方向を正、`corridor_z`は高さです。別の廊下やセンサー設置位置で使う場合は、その環境の録画から校正をやり直してください。

### ZED–LiDAR間の初期外部変換

Livox Mid-360Sの点群座標をZEDの左目カメラ座標へ変換する初期値は、`tracking/lidar_to_zed_left_eye.json`に保存します。現在の値はZED SDKの標準`IMAGE`座標系（x右、y下、z前方）で、LiDARの+Y軸がZEDの-X軸、下向き25度の取り付け、およびLiDAR原点のZED左目座標`[+60, -80, +20] mm`に基づきます。変換JSONと追跡CSVはm単位です。ZED SDKの既定単位はmmなので、SDK値を直接比較するときはmへ変換してください。

```text
point_zed = R × point_livox + translation

R = [[0,       -1,       0      ],
     [0.422618,  0,      -0.906308],
     [0.906308,  0,       0.422618]]
translation = [0.060, -0.080, 0.020] m
```

`track_people.py`で出力するCSVには、従来のLiDAR座標と廊下座標に加えて`zed_x/y/z`、`zed_vx/vy/vz`を出力します。これは初期取り付け値からの座標変換です。廊下での壁・床などを使ったZED–LiDAR間の精密化はまだ行っていないため、校正後はJSONの変換行列と平行移動を更新します。ZED SDKの`getPosition()`は標準で左目基準です。左右レンズの中央を原点にする場合は、実機校正値のステレオ変換から半ベースライン分を補正してください。SDKの座標系や単位を変更した場合は、この変換JSONも同じ設定に合わせます。

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
  recordings/livox/test_001_20260917_145114 \
  recordings/livox/test_001_20260917_145249 \
  --topic /livox/lidar --rate 1.0
```

RVizには`/tracking/points`（点群）と`/people/tracks`（人物マーカー）を表示します。人物の中心にID別の色付き球を置き、直近60フレームの軌跡を線でつなぎます。中心から伸びる矢印は進行方向と速度を表します。人物の横には各フレームの中心座標`x,y,z`（m）、水平速度`vx,vy`（m/s）、合成速度（m/s）を表示します。`min_track_displacement`により、移動が確認できたトラックだけをRVizに表示します。各bagの再生が終わった後も最後の表示を確認できます。

解析済みCSVから動きを再生する場合は、次のコマンドでブラウザ用の単体HTMLを生成して開きます。平面軌跡、フレーム送り・再生操作、人物ごとの速度時系列を同じ時間軸で確認できます。追加のPythonライブラリは不要です。

```bash
python3 scripts/replay_tracks_csv.py tracking/test_003_20260917_145451_tracks.csv
```

HTMLはCSVと同じ場所に`*_replay.html`として保存されます。空のフレーム時刻は前後のCSV時刻から補間し、先頭部分など補間できない区間は既定10 FPSで補います。FPSを変更する場合は`--fps 20`、HTMLだけ生成してブラウザを開かない場合は`--no-open`を指定できます。
カメラ映像と一緒に見る場合は、同じ試行のSVO2を`--svo`で指定します。CSV・SVOの試行名が一致することを確認し、対応するLiDAR bagを自動推定して、記録タイムスタンプの開始差を補正してからHTMLと`ZED_Explorer`を起動します。bagの場所を明示する場合は`--sync-bag`を指定します。

```bash
python3 scripts/replay_tracks_csv.py \
  tracking/test_003_20260917_145451_tracks.csv \
  --svo recordings/zed/test_003_20260917_145451.svo2
```

ZED_Explorerの再生速度はアプリ側で操作します。`--rate`はHTMLグラフの速度に適用されるため、カメラ映像と合わせる場合は通常の1倍速を使ってください。HTMLは既定の`--coordinate-frame auto`で校正済みの廊下座標を優先します。生のLiDAR座標で表示する場合は`--coordinate-frame livox`、廊下座標を必須にする場合は`--coordinate-frame corridor`を指定します。ZED画像の左右へ軌跡を重ねるには、別途ZED-LiDAR間の外部パラメータが必要です。`--zed-explorer`で実行ファイルを上書きできます。


点群を確認したい場合は、カメラ映像の代わりに解析に使ったrosbagを`--bag`で指定すると、RVizに点群とCSVの人物位置・速度マーカーが表示されます。ブラウザの速度グラフも指定した再生速度で自動再生します。

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/replay_tracks_csv.py \
  tracking/test_003_20260917_145451_tracks.csv \
  --bag recordings/livox/test_003_20260917_145451 \
  --rate 1.0
```

CSVのマーカーは点群メッセージのタイムスタンプでbagと対応付けます。HTML側のシーク・一時停止操作はRVizの点群再生には連動しません。

追跡処理は`config.json`の`tracking`ブロックで調整します。主な設定は次のとおりです。

- `roi`：センサー座標系で処理する範囲。廊下以外ではここを対象空間に合わせます。
- `output_coordinate_frame` / `corridor_frame_file`：追跡CSVへ廊下中心座標を追加する設定。校正ファイルは`estimate_corridor_frame.py`で作成します。
- `ground_mode` / `ground_z`：床面の設定。現在の`livox_frame`は床が概ね`z=0`のため`fixed`を既定にしています。床の高さが未知なら`auto`を試してください。
- `background_mode`：`temporal`で初期フレームから壁・家具などの静的点群を除去します。背景が動く場所では`none`にします。
- `cluster_*`：人物候補の大きさ・密度。点が疎な場合は`min_cluster_points`を下げ、候補が分断される場合は`cluster_tolerance`を上げます。`far_detection_start`以遠では、遠距離の点密度低下を考慮して`far_min_cluster_points`、`far_min_cluster_width`、`far_min_cluster_depth`を使用します。
- `cluster_background_voxel_size` / `cluster_background_min_presence`：点単位の背景除去を通過した壁・設備の候補を、クラスタ中心の滞在率でも除去します。
- `max_match_distance` / `max_track_speed` / `max_missed_frames`：歩行速度や遮蔽に応じたID追跡設定です。
  `max_match_distance`は一時的な点群中心のずれを同じIDへ再接続する距離ゲートです。
- `min_track_displacement`：この距離以上の移動が確認できるまでRVizには表示しません。静止物の誤検出を抑える設定で、静止人物も追跡したい場合は`0`にできます。
- `min_motion_consistency`：連続検出区間の移動量と経路長の比率です。方向が頻繁に反転する静的クラスタを人物として確定しないために使います。
- `motion_consistency_window`：移動の一貫性を評価する連続フレーム数です。短いクラスタ揺らぎを確定しないため、既定値は15です。

`background_mode: temporal`では、既定で最初の30点群フレームを背景学習に使うため、その間は人物候補を出力しません。開始直後から追跡したい場合は`background_mode: none`にするか、`background_learning_frames`を短くしてください。床の高さやセンサーの向きが録画ごとに異なる場合は、`ground_z`と`roi`を点群座標系に合わせて変更します。

これは点群だけを使う汎用的な幾何ベースのベースラインです。人数の上限で候補を削るのではなく、点単位の背景除去、候補クラスタの空間的な滞在率、移動量による確認を組み合わせています。壁や人が密着する、長時間遮蔽される、人物同士が交差する場合はIDの入れ替わりや見失いが起こり得るため、CSVの`status`、`confidence`、`detected`も併せて確認してください。より高い精度が必要になった場合は、この出力を教師データとして、人物検出モデルやセンサー間の座標変換を追加できます。

## コード構成と回帰確認

| ファイル | 主な役割 |
| --- | --- |
| `scripts/record_zed.py` | 設定読み込み、Livox接続確認、ZEDとrosbag2の録画・保存確認 |
| `scripts/replay_zed_lidar.py` | ZED・rosbag2・RViz2の同時再生と子プロセス管理 |
| `scripts/playback_timeline.py` | SVO2とrosbag2の記録時刻を読み、再生開始差を計算 |
| `scripts/track_people.py` | 点群の前処理、人物候補の検出、ID追跡、CSV・マーカー出力 |
| `scripts/corridor_frame.py` / `scripts/estimate_corridor_frame.py` | 廊下座標の推定と校正ファイルの保存 |
| `scripts/replay_tracks_csv.py` / `scripts/tracking_replay.html` | 追跡CSVの整理と単体HTMLの生成・表示 |
| `scripts/replay_tracking_rviz.py` / `scripts/convert_livox_custom.py` | RVizでの追跡再生とCustomMsg点群の変換 |

実機を使わずに主要な保存名、記録時刻、CSV補間、ID維持、座標変換を確認できます。追跡関連の確認にはNumPyが必要です。

```bash
python3 -m compileall -q scripts tests
python3 -m unittest discover -s tests -v
```

ZEDカメラ、Livox、ROS 2、RViz2を起動する一連の操作は実機環境で確認してください。
