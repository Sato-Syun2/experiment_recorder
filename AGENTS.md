# ZED + Livox 実験記録・解析プロジェクト

## プロジェクト概要

このリポジトリは、歩行実験の録画、試行順管理、ZED 2i の SVO2 と Livox Mid-360S の ROS 2 rosbag2 の再生・解析を行う。主な実行環境は Ubuntu、Python 3、ROS 2 Jazzy、ZED SDK 5.4 系。ZED は `ZED_Explorer` を使い、LiDAR は `livox_ros_driver2` と `/livox/lidar`・`/livox/imu` を使う。

録画の入口は `scripts/record_zed.py`。基本設定は `config.json`。試行順・条件・役割は生成済みの `run_order.json` を唯一の参照元とし、Excel の内容を変更したときは録画前に `scripts/build_run_order.py` で再生成する。試行順や教示を個別のスクリプトへ重複記載しない。

## 主要ファイル

- `README.md`: 録画、再生、人物追跡、設定の使い方。
- `config.json`: 録画先、ROS 2、再生、点群追跡、座標校正の設定。
- `run_order.json`, `inputs/`, `reports/`: 試行計画、入力表、生成した監査資料。
- `scripts/record_zed.py`: ZED/LiDAR録画ランチャー。Livoxが使えない場合もZED録画は続ける。
- `scripts/track_people.py`: LiDAR点群から人物候補を検出し、ID付きCSVと必要に応じRVizマーカーを出力。
- `scripts/replay_zed_lidar.py`: SVO2とLiDAR rosbag2の再生。
- `scripts/export_svo_review.py`: Windows上のZED SDKでSVO2から時刻付き画像・深度・要求時のIMUデータを出力。
- `scripts/calibrate_zed_lidar_aruco.py`: 複数姿勢のArUcoパネル面とZED/Livox IMU重力方向からZED–LiDAR間の6自由度変換候補を作成。
- `scripts/compare_zed_lidar.py`: ZED画像/深度の前景候補とLiDARトラックを照合し、HTML・CSV・JSONを生成。
- `tracking/lidar_to_zed_left_eye.json`: LivoxからZED左目座標への外部変換。現在は初期取り付け測定値で、精密校正値ではない。
- `tracking/corridor_frame_livox_level.json`: 水平化したLiDAR点群から推定した廊下座標系。

## 解析上の前提と注意

- LiDARの確定検出はCSVの `status=confirmed` と `detected=1` を確認する。未確定の候補トラックを人物検出として数えない。
- `x,y,z` はLiDAR元座標、`level_x,level_y,level_z` は重力水平化した座標、`corridor_x/y/z` は廊下座標、`zed_x/y/z` はZED画像座標への変換値。距離と単位の解釈を混ぜない。位置・深度はm、ZED SDK既定値はmmの場合がある。
- `gravity_alignment` と廊下校正値は設置姿勢に依存する。センサーを移動したら記録を取り直して校正し、過去の値を別設置場所へ流用しない。
- 今回確認した `20261013_group001/recording_001` は、廊下を1人で往復した試験。水平化後のLiDAR確定トラックID 45は最大水平距離約12.92 m、最大時のクラスタ点数は9点。40 mの製品測距仕様を人物の検出・追跡距離と解釈しない。
- この試行のZED/LiDAR時刻対応は `LiDAR time ≈ ZED elapsed + 1.675882 s`。これはこの録画の値で、別録画へ固定適用しない。
- 現在のZED/LiDAR比較スクリプトは、人物が写る前の空の廊下を背景として画像・深度の移動前景を抽出する。ZED SDKのBody Trackingや意味的な人物検出結果ではない。比較ページの3D差は異なる領域中心（深度候補中央値とLiDARクラスタ重心）の差で、正解位置に対する測位誤差ではない。
- LiDAR→ZEDの投影には初期外部変換を使う。画像上の上下ずれを見つけた場合は、まず時刻同期と外部校正を確認する。見た目だけに合わせて校正JSONを書き換えない。
- 比較HTMLは同名の `_frames/` サブフォルダー内の画像を参照する。HTMLだけ移動・配布せず、フォルダーごと保つ。

## 共有フォルダーのルール

共有データのルートは Linux 側の `/data/experiment_data/`。Windowsから同じ共有領域にアクセスする場合は、今回使った `\\tsclient\_data_experiment_data\` の形式を使う。録画ファイルは次の構成。

```text
/data/experiment_data/
├── recordings/<YYYYMMDD>_group<NNN>/recording_<NNN>/
│   ├── recording_<NNN>.svo2
│   ├── recording.json
│   └── lidar/                 # metadata.yaml と rosbag2 ファイル
├── scripts/                   # 外部PC（Windows/ZED SDK）で実行するスクリプト
└── recording_001_svo_review/  # 今回の画像・深度・必要時のIMU書き出し例
    ├── camera.json
    ├── frames.csv
    ├── imu.csv                 # --include-imu 指定時
    ├── frames/
    └── depth/
```

- 録画データは原本として扱う。依頼がない限り共有フォルダー内の録画を削除、移動、改名、上書きしない。解析出力はリポジトリの `tracking/` または `reports/`、あるいは録画と分けた新規フォルダーへ保存する。
- 新しい録画は既存フォルダーを使い回さず、ランチャーの採番で新しい `recording_NNN` を作る。試行後のメタデータや条件ラベルを手作業で既存録画へ書き戻さない。
- 外部PCで実行するZED SDK用スクリプトは、ユーザー指定により `/data/experiment_data/scripts/` に置く。Windows側で共有フォルダーが見える場合は `\\tsclient\_data_experiment_data\scripts\` を使う。SVO2は外部PCのローカルパスから読み、レビュー用出力や解析結果は共有フォルダーの新しい専用ディレクトリへ出す。
- `/data/experiment_data` が現在の実行環境から読み取り専用、または書き込み権限外の場合は、権限を回避しない。作業用スクリプトや出力をリポジトリ内に作って、共有先へのコピーが必要なことをユーザーに伝える。
- 共有先にファイルを置く場合は、入力原本と解析出力を混在させない。元SVO2・bagを含む大容量データを無断で複製しない。処理対象と出力先を明記し、既存名と衝突しない名前にする。
- Windows PowerShellの複数行継続はバッククォート `` ` ``。Bash用の `\` はPowerShellでは使わない。Pythonパッケージは実行対象と同じPythonに `python -m pip` で入れる。SDKバインディングが見つからない場合は `python -c "import sys; print(sys.executable)"` と `py -0p` で実行環境を確認し、ZED SDKに合うPython版のAPIを導入する。

## 代表的な処理手順

### LiDAR人物追跡

```bash
python3 scripts/track_people.py \
  /data/experiment_data/recordings/20261013_group001/recording_001/lidar \
  --topic /livox/lidar
```

追跡結果は既定で `tracking/<bag名>_tracks.csv` に保存する。録画に合わせた重力水平化・廊下座標系・点数しきい値を確認し、別環境のために `config.json` を変更した場合は設定差を記録する。

### ZED SVO2レビューの書き出し

WindowsのPowerShellで、SVO2ファイルをローカルから指定する。改行する場合はPowerShellのバッククォートを使う。例:

```powershell
python "\\tsclient\_data_experiment_data\scripts\export_svo_review.py" `
  "C:\data\recording_001.svo2" `
  --output "\\tsclient\_data_experiment_data\recording_001_svo_review" `
  --every-seconds 0.5 --include-depth
```

必要なPython依存（例: OpenCV）を入れ、`pyzed.sl` が import できるZED SDK対応のPythonを選ぶ。`get_sdk_version()` が使えないことだけではSDKバインディングの失敗と判断しない。

### ZED深度・LiDAR比較

レビュー出力が `/data/experiment_data/recording_001_svo_review` にあり、追跡CSVが作成済みなら:

```bash
python3 scripts/compare_zed_lidar.py \
  --review-dir /data/experiment_data/recording_001_svo_review \
  --tracks tracking/20261013_group001_recording_001_gravity_aligned_tracks.csv
```

既定出力は `tracking/20261013_group001_recording_001_zed_lidar_comparison.html` と、同名CSV/JSON、`_frames/` 画像フォルダー。HTMLを開くときも `_frames/` を隣に残す。時刻オフセットを再利用するときは録画ごとに同期を確認する。

## 変更時の進め方

- 変更前に関連コードとREADMEの記述を読む。汎用処理と録画固有の校正・出力を分ける。
- 録画・試行計画・センサー設定に関わる変更は、変更対象と影響をREADMEまたは解析レポートへ反映する。
- 再解析結果は入力録画、設定値、時刻オフセット、スクリプト名が分かる形で保存する。
- 実機のZED、Livox、ROS 2、RVizの起動を伴う確認は、実機操作であることを明記する。テストは依頼されたときに実行し、既存テストを使う場合はREADME「コード構成と回帰確認」の手順に従う。
