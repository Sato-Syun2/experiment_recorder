# 録画003：ポール設置下の歩行確認（LiDAR初期解析）

2026-10-22。ユーザー申告：録画003はポールを置いた状態で歩行している。元の試行メタデータは変更せず、この用途で確認する。

**移動点群は見えているが、現行のLiDAR追跡では確定人物検出が0/273フレーム。画像・SDK骨格はまだ書き出されておらず、正常観測やポールによる遮蔽・誤対応は最終判定できない。**

[点群と候補の再生](index_preliminary.html) ／ [高さ帯別の点群・候補のグラフ](lidar_preliminary.png) ／ [集計](metrics_preliminary.json)。

## LiDARで確認したこと

- 保存bagは273フレーム、ヘッダー時刻の重複・逆行0。約27.201秒。
- 廊下中央付近の低い高さ帯には、廊下方向X **約2.875 m・4.825 m**に静止した強い帯がある。ユーザー申告のポールと対応する候補だが、映像でまだ同定していない。点群や静止候補を人物数に数えない。
- 高さ0.95〜2.2 mの帯には、手前から **約9 mまで進んで戻る**移動点群が見える。これは生の点群分布であり、人と確定した追跡結果や測量した歩行距離ではない。録画002の15 m条件をそのまま当てはめない。
- 全区間を現行の検出・確定閾値で処理し、作成IDは122。**`status=confirmed` かつ `detected=1` は0件・0フレーム**。未確定候補には静止物と移動物の両方が混じる。122を人数に数えない。
- 中央の移動に対応する可能性があるID 38・64・100の最長連続観測は、それぞれ **14・14・9フレーム**。現行の動きの整合性判定は連続した15観測の履歴を必要とするため、これらの候補はこの条件を満たさない。閾値を緩めて確定へ変える処理は行っていない。
- 静止した帯付近では、ID 61・109等に位置がほぼ一定の候補がある。ID 8は静止帯から移動する点群との関連が疑われるが、映像なしに「ポールから人へIDが移った」と断定しない。候補別の件数・距離・時間・連続観測は [candidate_diagnostics.csv](candidate_diagnostics.csv)。

生の点群に人らしい移動が見えることと、人物の確定追跡が成功していることを分けて扱う。ポールなしの録画002とは経路・距離・歩行位置・初期化条件も同一とは確認できておらず、検出不成功の原因をポールだけに帰属できない。

## 設置と座標

録画003の先頭30点群とIMUから床・壁・廊下座標を独立に推定した。人物軌跡やポールの公称距離に合わせて軸・原点を調整していない。

録画002との違い:

|項目|差|
|---|---:|
|LiDAR IMU上方向|0.060°|
|床面法線|0.074°|
|床面までの距離|+0.0017 m|
|廊下幅|−0.0017 m|
|元LiDAR座標での廊下方向軸|0.078°|

LiDARの床・壁の大きな変化は見られないが、センサーの廊下方向の並進やZEDカメラの向きの不変性を保証しない。**003のZED校正・IMUは未確認**で、002の骨格結果や校正をそのまま003へ適用して正常判定しない。運用中の `config.json` と `tracking/*.json` は変更していない。

## Windowsでの追加書き出し

このUbuntu環境には `pyzed` がない。003の画像・深度・IMU・骨格をまとめて取得するランナーを `/data/experiment_data/scripts/` に新規配置した。ZED SDKが使えるWindowsのPowerShellで実行する。SSDパスは前回と同じ構成を想定している。

```powershell
python "\\tsclient\_data_experiment_data\scripts\run_group003_recording003_pole_walk_20261022.py" `
  "F:\20261022_group003\recording_003\recording_003.svo2"
```

次の3段階を別プロセスで実行する。

1. 全区間の補正済み画像・m単位の深度・IMUを約0.2秒間隔で書き出す。
2. 全区間のZED Body Trackingを書き出す。これは往路からの追跡履歴を持つ条件。
3. LiDARの移動帯から提案した折返し後の区間へシークし、位置・骨格追跡の開始前に移動した状態で、別の新規SDKプロセスで接近だけを再推定する。往路のSDK履歴を引き継がない。

接近窓の提案は **LiDAR先頭から16.8〜25.1秒**の未補正の絶対ヘッダー時刻で選ぶ。003の時計offsetはまだ推定しておらず、画像の到着後に本当に折返し後になっているか、ポールのどちらを通ったか等を確認する。これは映像で確定した人物存在区間ではない。SDKの実フレーム時刻を使って窓を作り、フレーム番号×fpsで時刻を合成しない。

BODY_34 / HUMAN_BODY_ACCURATE / NEURAL / 最大30 m / confidence 35。原SVOはSSDから読み取り、新しい出力先を使う。実機のZED・LiDAR、ROSノード、RVizは起動しない。

```text
\\tsclient\_data_experiment_data\20261022_group003_recording_003_pole_walk_check\
  sdk_runs\<実行日時>\
    review\                 画像・深度・IMU・内部パラメータ
    body_full\              往復全体のSDK骨格・全フレーム台帳
    body_approach\          接近から新規開始したSDK骨格・全フレーム台帳
    window_approach.json     接近窓の全実タイムスタンプ
    run_complete.json        全書き出し完了（精度合格の意味ではない）
```

失敗時は処理段階と例外を `run_failed.json` に記録し、途中出力を保持する。完了しても人物の正常観測と自動判定しない。この段階ではSDKを書き出す準備までで、003のSDKはまだ実行していない。

## 画像・骨格の到着後に確認すること

- 人とポールが別の対象として見えているか。静止帯と候補IDがどちらに対応するか。
- ポールと重なる前・最中・通過後に、LiDAR人物候補の途切れ、ID切替、ポールへの誤対応があるか。確定と未確定を分ける。
- ZEDの人物・骨格がポールへ移らないか。人物ID・実画像への重なり・足の画角外・関節confidence・床高・骨長を確認する。
- 同じ003の全区間追跡と接近からの新規追跡を同時刻で比べ、履歴の影響も分ける。002との比較では同じ条件であるとは断定しない。
- 今回のZED IMU・床・壁を確認し、別録画のclock offsetや骨長基準を固定流用しない。

## ファイルと再現

入力：`/data/experiment_data/recordings/20261022_group003/recording_003/lidar/`。SVO原本と既存メタデータは変更・複製していない。新規解析はリポジトリの `reports/20261022_group003_recording_003_pole_walk_check/`、共有用資料は共有領域の同名専用フォルダー。点群キャッシュはリポジトリ内の新規出力だけに保存する。

実施したのは保存bagの読取・解析・描画。ユニットテストは依頼されておらず実行していない。

```bash
source /opt/ros/jazzy/setup.bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python reports/20261022_group003_recording_003_pole_walk_check/prepare_lidar.py
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/track_people.py \
  /data/experiment_data/recordings/20261022_group003/recording_003/lidar \
  --config reports/20261022_group003_recording_003_pole_walk_check/analysis_config.json \
  --output reports/20261022_group003_recording_003_pole_walk_check/tracks.csv
OPENBLAS_NUM_THREADS=1 .venv/bin/python reports/20261022_group003_recording_003_pole_walk_check/build_preliminary.py
```

再実行時は新規専用出力先を用意する。来歴は `geometry.json`、`tracks.metadata.json`、`preliminary_manifest.json`、`delivery_manifest.json` に保存。
