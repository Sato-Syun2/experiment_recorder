# 録画003：骨格と廊下内の歩行を再生する

## 現在の状態

2026-10-22追記: 外部SDK PCの出力（全899フレーム、796件の骨格観測）が到着した。[骨格付き再生と確認結果](verified_20261022/README.md) を参照。SDK OKの757件中392件で、折返し後の脚・胴体の形状に大きな不整合がある。新しい再生画面では該当骨格を既定で非表示にし、元データは紫で切り替えて確認できる。

[表示を拡大した最新の骨格付き再生画面](display_fix_20261022/index.html) を開く。頭・骨盤の高さと床Z=0の目盛りを追加し、上面経路図を折りたたんだ。最新版は形状不整合も紫の元データとして初期表示する。検証時の旧表示は `verified_20261022/index.html`、骨格到着前の表示はルート `index.html` に保持する。以下のコマンドは初回書き出し時の手順であり、今回の表示修正のために再実行する必要はない。

## Windows・ZED SDK対応PCの実行コマンド

既存のZED SDK 5.4 / `pyzed` / NumPy / OpenCVが利用できるPythonを使う。PowerShellで実行する。入力はレビュー書き出し時のメタデータに記録されたローカルSVO2パス。保存場所を変えた場合は2行目を変更する。

```powershell
python "\\tsclient\_data_experiment_data\scripts\run_recording003_skeleton_replay_20261021.py" `
  "C:\Users\sawaragilab2024\Desktop\sato_shun\svo\recording_003.svo2"
```

スクリプトはSVO2を読み取るだけで、カメラ・LiDARの実機やROSノードは起動しない。SVO2を共有経由で読み込む指定は拒否する。録画原本をコピーしない。

出力先は新規作成される次の場所。日時を毎回付け、既存結果を上書きしない。

```text
\\tsclient\_data_experiment_data\20261021_group002_recording_003_skeleton_replay\exports\
  body_<実行日時>\
    camera.json
    frames.csv
    bodies.csv
    preview_frames\
    contact_sheet.jpg
  walk_skeleton_<実行日時>.html
  walk_skeleton_<実行日時>.manifest.json
```

SDKは全SVOフレームで骨格を推定する。`--preview-period-s 0.5` は確認画像の保存間隔だけであり、骨格を0.5秒間隔へ間引く指定ではない。設定はBODY_34、HUMAN_BODY_ACCURATE、body fitting / tracking / segmentation有効、深度NEURAL、最大距離30 m、人物検出confidence 35。設定値は精度保証ではない。

`pyzed`等を別のPythonへ導入している場合は、コマンド先頭の `python` をそのPython実行ファイルへ置き換える。依存を追加する場合は同じ環境の `python -m pip` を使う。SDKエラーが出た場合はそこで停止し、完成した骨格再生として扱わない。

## 再生画面

- 斜め上、上面、側面、正面の視点を切り替える。ドラッグで回転、ホイールで拡大縮小。
- 時刻スライダー、再生速度0.25〜2倍、人物を追う視点。
- SDK骨格をZED IMAGE座標からLiDAR元座標、水平化座標、廊下座標へ変換して描く。録画002由来の校正候補を固定使用する。
- 骨格はSDKが出力した位置へ表示し、LiDARマーカーに吸着・移動させない。LiDARの確定観測と未確定候補は別に表示する。
- 初期状態ではtracking_stateがOKの骨格のみ、関節confidence 25以上のみを表示。低信頼度の関節を選んで表示すると赤で区別する。confidenceなし・非有限の関節は描画しない。
- 欠測関節は補完せず、直近観測を長時間保持しない。時刻差65 ms以内の最寄りSDKフレームにデータがある場合だけ表示する。
- 2D骨格は画像時刻に最も近い同じ録画の骨格から描く。映像は約10 Hz、骨格は原SVOのフレーム間隔なので、3Dと画像の表示時刻差を明記する。
- 骨格CSVが既にある場合は、画面の「骨格書き出しフォルダーを読み込む」でcamera.json / frames.csv / bodies.csvのフォルダーを選ぶこともできる。データは外部へ送信しない。

タイムスタンプ・カメラシリアル・単位・BODY_34形式を確認し、別録画の骨格は拒否する。出力は元SVOの全899フレームを期待し、不一致の場合は部分データであると明記する。

## 測定上の注意

時計差は録画003について未確定のため、生の絶対時刻を使用する。別録画のoffsetを適用していない。ZED骨格とLiDARの部位・代表点の違い、校正候補、時計差により位置がずれる場合がある。画面の壁・高さガイドは表示用の概略で、厳密に測量した廊下モデルではない。脚・足首の信頼度が低い場合は表示が欠ける。歩容・関節角・歩幅の精度を保証するものではない。

LiDARの検出継続性と大まかな経路の結果は [前の検証レポート](../20261021_group002_recording_003_walk_assessment/README.md)。録画003では確定追跡に約11.4秒の途切れがある。骨格が到着しても、未確定LiDAR候補を確定検出に数えない。

## 構成と確認範囲

`build_walk_skeleton_replay.py` はPython標準ライブラリのみでHTMLを生成する。実際の骨格推定にはSDK対応PCが必要。表示はブラウザーのCanvasで行い、外部CDNやサーバーは不要。HTMLは `assets/` との相対位置を保って共有する。`exports/` のHTMLを単独で別の場所へ移動しない。

準備物はリポジトリのこのフォルダーと、共有領域の `/data/experiment_data/20261021_group002_recording_003_skeleton_replay/` に保存。SDK用スクリプトは共有 `/data/experiment_data/scripts/` に別名で追加し、既存スクリプトを上書きしていない。新しく生成した小さなレビュー画像以外に、SVO2やbagの大容量複製はしていない。実機操作・ユニットテストは実行していない。
