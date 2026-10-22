# プログラム軌跡の人手確認手順

LiDAR追跡プログラムの出力を、録画映像と合わせて人が確認するための手順。人手確認と、独立した位置真値による精度評価を分けて記録する。

## 1. 追跡結果を作る

対象bagと出力先を明示する。次は録画001の例なので、別録画ではbagと出力名を合わせて変更する。出力名は録画ごとに新しくする。

```bash
source /opt/ros/jazzy/setup.bash
BAG=/data/experiment_data/recordings/20261013_group001/recording_001/lidar
TRACK_DIR=tracking/manual_review_20261013_recording_001
python3 scripts/track_people.py \
  "$BAG" \
  --config config.json \
  --topic /livox/lidar \
  --output "$TRACK_DIR/tracks.csv"
```

以降のコマンドも同じシェルで実行する。別シェルを使う場合は `BAG`、`TRACK_DIR`、`SVO` を定義し直す。

追跡CSVのほかに同じ名前の `.frames.csv`（検出のない処理フレームを含む）と `.metadata.json`（入力・設定・コードの来歴）が作られる。3点を揃えて保管する。途中で止まった処理は、完了済みとして評価しない。

## 2. 軌跡を表示する

```bash
python3 scripts/evaluate_tracking.py \
  --tracks "$TRACK_DIR/tracks.csv" \
  --frames "$TRACK_DIR/tracks.frames.csv" \
  --output-dir reports/manual_review_20261013_recording_001/overview
```

生成された `index.html` を開く。必要なら単体再生HTMLも作る。

```bash
python3 scripts/replay_tracks_csv.py \
  "$TRACK_DIR/tracks.csv" \
  --output "$TRACK_DIR/replay.html"
```

映像と一緒に見る場合は同一録画のSVO2とbagを指定する。ZED_Explorerが起動する。

```bash
BAG=/data/experiment_data/recordings/20261013_group001/recording_001/lidar
SVO=/data/experiment_data/recordings/20261013_group001/recording_001/recording_001.svo2
TRACK_DIR=tracking/manual_review_20261013_recording_001
python3 scripts/replay_tracks_csv.py \
  "$TRACK_DIR/tracks.csv" \
  --svo "$SVO" \
  --sync-bag "$BAG" \
  --config config.json
```

別の時計や時刻補正が必要な録画は、補正値を独立に確認してから使う。録画開始時刻の差だけを時計offsetとみなさない。

## 3. 人が見る項目

`tracks.csv`では実観測と予測を分ける。確定検出として数えるのは `status=confirmed` かつ `detected=1` の行だけ。`predicted` は欠測中の推定位置で、LiDARがその時点で人物を検出した証拠ではない。

対象録画を再生し、少なくとも次を確認する。

1. **人物との対応:** 各IDが映像上で誰に対応するか。計画上の役割・実施条件は `run_order.json` だけで決めず、実際の映像も見る。
2. **検出と欠測:** 人がいるのに検出がない区間、人物がいないのに軌跡が出る区間、予測だけでつながっている区間。
3. **IDの連続性:** 交差・追越し・遮蔽の前後でIDが入れ替わる、分裂する、別人へ移る箇所。
4. **位置の自然さ:** 瞬間移動、急な横跳び、壁越しの位置、廊下範囲外への飛び出し。座標系と単位も確認する。
5. **端点と折返し:** 進入・退出、停止、折返し付近の見失いと予測。処理開始時の背景学習区間も確認する。

候補箇所を記録するときは、録画名、track ID、開始・終了時刻、映像で見た人物、問題の種類、確認者、確認日、同期方法を残す。例: `track_id, start_s, end_s, observed_person, issue, note, reviewer, date`。原CSVは書き換えず、別ファイルへ記録する。

人物IDと被験者の対応・役割を評価画面へ付ける場合は、実際の映像を確認してから注釈JSONを作る。計画上の役割は `run_order.json` から読み、録画JSONと計画の一致を確認する。

注釈JSONの形式は次のとおり。`recording` は対象録画ディレクトリの絶対パス、`track_participants` は映像で確認したトラックIDと被験者の対応とする。

```json
{
  "recording": "/data/experiment_data/recordings/YYYYMMDD_group001/recording_NNN",
  "actual_behavior_verified": true,
  "track_participants": {"12": "subject_1", "28": "subject_2"}
}
```

評価コマンドに `--roles annotation.json --recording-json /path/to/recording.json --run-order run_order.json` を追加する。役割・条件は注釈JSONへ重複記載せず、録画の計画ハッシュに一致する `run_order.json` から取得する。IDが途中で変わった場合は対応の根拠も別の注釈記録に残す。

## 4. 独立真値がある場合だけ数値評価する

プログラムの軌跡やZED推定値を正解ラベルに流用しない。位置誤差を評価する場合は、測量・独立カメラ等で位置を測り、身体上の基準点、廊下座標原点、単位、計測誤差を事前に定義する。

`labels.csv` はLiDAR時刻に合わせ、独立計測した廊下位置をmで保存する。各評価時刻にいる人物を全員記録し、空の場面は空の `person_id` で表す。

```csv
timestamp_ns,person_id,x_m,y_m,vx_mps,vy_mps
1791863426622916075,participant_A,4.0,-0.2,,
1791863426722916075,,,,,
```

```bash
python3 scripts/evaluate_tracking.py \
  --tracks "$TRACK_DIR/tracks.csv" \
  --frames "$TRACK_DIR/tracks.frames.csv" \
  --labels /path/to/independent_labels.csv \
  --reference-note "計測法、身体基準点、座標校正、評価分割、注釈者" \
  --gate-m 0.75 --time-tolerance-s 0.049 \
  --output-dir reports/manual_review_20261013_recording_001/reference_evaluation
```

`--gate-m`と時刻許容幅は参照計測・センサー周期に合わせて決め、結果と一緒に残す。出力の `metrics.json`、`reference_matches.csv`、`index.html` を確認する。位置誤差は対応できた標本に対する値なので、欠測率・偽陽性・ID Switchと併記する。全処理フレームは `.frames.csv` から数え、追跡CSVの行数を検出率の分母にしない。

映像だけでIDの対応を確認できても、独立なメートル位置がなければ位置MAE/RMSEは出せない。`pairs.csv` の人物間距離は同時刻の確定クラスタ代表点間距離で、身体表面間距離や連続時間の最接近距離ではない。`kinematics.csv` の微分値は無平滑で、動作開始時刻・回避量の確定値ではない。
