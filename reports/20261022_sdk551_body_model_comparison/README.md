# SDK 5.5.1：NEURAL / BODY_34 のモデル・世代比較

2026-10-22。指定の **HUMAN_BODY_MEDIUM / GEN_2** を第一候補として、同じSVO2・元画像時刻・追跡初期化条件で比較する準備。

**新モデルのSDK推論はまだ実行していない。優劣の判定は未完了。** Ubuntu側には `pyzed` がなく、推論はWindows SDK PCで行う。過去の出力から作った [比較基準の集計](baseline_reference/index.html) は確認済み。

## 実行コマンド

今回の比較では、前の003用ランナーの代わりに、SDK **5.5.1と対応するPythonバインディング**が使えるWindowsのPowerShellで次を実行する。指定するのはSVOそのものではなく、録画002・003が入ったSSDのグループフォルダー。

```powershell
python "\\tsclient\_data_experiment_data\scripts\run_sdk551_body_model_comparison_20261022.py" `
  "F:\20261022_group003"
```

2本のSVO2をSSDから読み、計15条件を逐次実行する。初回は各モデルの取得・最適化が入る場合がある。新規日時フォルダーへ保存し、完了後に比較HTMLを開く。GPUを使う保存録画の解析であり、ZED/Livox実機・ROSノード・RVizは起動しない。

```text
\\tsclient\_data_experiment_data\20261022_sdk551_body_model_comparison\
  runs\<実行日時>\
    recording_002\approach\<モデルID>\      5条件
    recording_003\full\<モデルID>\          5条件
    recording_003\approach\<モデルID>\      5条件
    recording_003\review\                    画像・深度・IMU
    comparison.json                          全指標
    index.html                               比較表・骨長/位置グラフ・画像一覧
    case_results.json                        条件別の完了・失敗
    run_complete.json                        処理状態（精度合格ではない）
```

必要なら対象を限定できる。例えば、まず指定モデルと新旧ACCURATEをポールのある003で比較する場合:

```powershell
python "\\tsclient\_data_experiment_data\scripts\run_sdk551_body_model_comparison_20261022.py" `
  "F:\20261022_group003" `
  --recordings 003 `
  --models medium_gen2 accurate_gen2 accurate_gen1
```

この指定では003の全区間・接近の計6条件を処理する。限定した実行を全15条件の完了とは扱わない。

## 比較する設定

深度 **NEURAL**、骨格形式 **BODY_34**、body fitting / tracking / segmentation有効、最大30 m、検出confidence 35、プレビュー0.2秒を固定する。

|モデルID|骨格モデル|世代|役割|
|---|---|---|---|
|medium_gen2|HUMAN_BODY_MEDIUM|GEN_2|今回の指定|
|accurate_gen2|HUMAN_BODY_ACCURATE|GEN_2|同世代でモデルを比較|
|medium_gen1|HUMAN_BODY_MEDIUM|GEN_1|MEDIUMで世代を比較|
|accurate_gen1|HUMAN_BODY_ACCURATE|GEN_1|従来モデル系を新SDKでも再実行|
|fast_gen1|HUMAN_BODY_FAST|GEN_1|高速モデルとの比較|

公式仕様ではGEN_2はMEDIUM・ACCURATEのBODY_18/BODY_34に対応し、FASTはGEN_1。非対応の組合せをGEN_2として実行しない。SDKは非対応の場合に世代を自動的に切り替えることがあるため、この比較ではモデル・世代を明示指定し、`get_body_tracking_parameters()` で読み戻した設定も照合する。読み戻し不能・不一致は成功扱いしない。

- [公式：Body Tracking API](https://docs.stereolabs.com/docs/development/zed-sdk/modules/body-tracking/using-the-api)
- [公式：SDK 5.5 / 5.5.1 リリースノート](https://docs.stereolabs.com/docs/development/zed-sdk/release-notes/5-x/5-5)
- [公式：世代を上書きする環境変数の説明](https://docs.stereolabs.com/docs/integrations/ros-2/zed-stereo-node)

`ZED_SDK_BODY_TRACKING_MODEL_GENERATION` がユーザー環境にあっても、比較用の子プロセス環境からだけ除去し、API引数を使う。親の環境変数は変更しない。実SDK版は `sl.Camera.get_sdk_version()`、Pythonバインディング版は別フィールド、深度精度・位置追跡モード・予測timeout等も実行メタデータへ記録する。SDK 5.5.1以外では止める。

## 初期化と時間窓

- **録画002・接近**：以前の解析と同じ元フレーム216〜629、全414フレーム。実タイムスタンプを以前の `window.json` と全件照合する。
- **録画003・全区間**：最初から最後まで。同じ全元フレームを全モデルで処理する。空の区間や往路の追跡履歴も含む。
- **録画003・接近**：LiDAR先頭から16.8〜25.1秒の絶対ヘッダー時刻を提案窓として使う。全モデルで同じ元フレームを処理する。画像到着前の提案であり、折返し後・人物の存在・ポール通過位置は映像で確認する。003の時計offsetはまだ推定していない。

各条件は別Pythonプロセス・別ZED Camera。元フレームへシークしてから位置追跡・骨格追跡を初期化し、モデル間・往路から接近へのSDK状態を引き継がない。全区間条件の内部では通常どおり履歴を保持する。

最初に深度・位置・骨格追跡なしでSVOの全実タイムスタンプを読み、各条件用manifestを固定する。フレーム番号×fpsで時刻を作らず、間違ったSVO・SDKによる時刻変更・フレーム欠落があれば失敗とする。近傍時刻だけで動体同期の精度を証明しない。

## 指標と優劣の判断

比較HTMLには、SDK OK出力率、両脚6関節のconfidence 25以上・画像内の割合、SDK OKのID数・最大欠落・単一IDの切替、骨長の変動、録画002のかかと高、SDK処理時間を並べる。同じフレーム時刻・同じ録画/範囲の中で比較し、002と003や全区間と接近の割合をそのまま比較しない。

**SDK OKは実検出率・関節精度の保証ではない。** SDKの予測でOKが維持される場合もある。主要IDは件数最大のIDを自動選択しており、人物と映像で照合済みという意味ではない。ポールを人と推定したモデルが高い出力率を得ても、優れているとはしない。

- 骨長は、各モデル自身の同じ固定近距離時間窓の中央値に対する比と、絶対長の範囲を併記する。002は原録画17〜19秒、003はLiDAR先頭21〜22.5秒相当。相対変動が小さくても、一様なサイズ誤りは見つからない。実測身体寸法ではない。
- 002の床高・廊下座標は前段で固定した候補を全モデルへ同じように適用し、モデルごとにフィットし直さない。003のZED床校正はまだ未確認なので、002の変換を流用して003の床下の足を判定しない。003の画像・深度・IMUを併せて書き出し、設置・ポール・骨格を後で確認する。
- 処理時間は最初の30フレームを除いた `grab + retrieve_bodies` のホスト実時間の中央値・90百分位。SVO復号・深度等も含み、GPU推論だけの時間やアプリ全体の実時間FPSではない。モデル初期化時間・CSV/画像出力を含む壁時計時間を別保存する。1回ずつの参考比較で、温度や初回最適化の影響まで統計的に除去した速度保証ではない。
- 位置・関節の独立真値がないため、confidence・床との位置関係・骨長・LiDARとの差だけから絶対精度の勝者を決めない。画像上で人・ポールの対応、関節の重なり、遮蔽前後を確認した上で用途ごとに選ぶ。

最終的には「概略経路を途切れず得る」「ポールとの誤対応を防ぐ」「脚・足の形状を安定させる」「処理負荷を下げる」の各点で比較する。全項目を無根拠に単一スコアへ潰さず、まず明らかな破綻や誤対応を除く。現在の `ranking_status` は画像照合待ち。

## 過去の結果をどう扱うか

録画002の `20261022_150530_153836/sdk` を改変せず、比較用の新規 `historical/recording002_sdk/` に小さなSDK出力を固定保存した。SVO・bagを新しく複製していない。

新しい集計でも、以前の結果と一致することを確認した:

- SDK OK：411/414（99.3%）。
- 両脚6関節confidence 25以上・画像内：307/414（74.2%）。
- 画像内・confidence 25以上のかかとの最小高：約−0.278 m。
- 同条件のかかとが−0.10 mを下回るフレーム：57件。

当時の `camera.json` の `sdk_version=5.4` は旧エクスポーターがPythonパッケージ版を記録したもので、実SDKのパッチ版・有効世代の読み戻しはない。旧出力とGEN_2の差だけではSDK更新・世代・モデルの影響を分離できない。**SDK 5.5.1上のACCURATE/GEN_1を対照として再実行**することが重要。同じSDK内でMEDIUM/GEN_1→GEN_2、ACCURATE/GEN_1→GEN_2、MEDIUM→ACCURATEを比較する。

録画003の過去SDK結果は、この準備時点では共有先に存在しなかった。未実行を検出ゼロと扱ったり、002の成績を003へ流用したりしていない。

## 実装と確認の範囲

- `scripts/analyze_svo_body_tracking.py`：モデル・世代オプション、設定読み戻し、実SDK版、フレーム別処理時間を追加。指定なしのモデルは以前と同じACCURATE、世代はSDK既定値のAUTO。再現比較では明示指定を必須にする。
- `scripts/run_body_model_comparison.py`：15条件を逐次・新規SDKプロセスで実行し、個別失敗を記録する。
- `scripts/compare_body_models.py`：同一時刻の台帳・人物行の整合性を確認して集計・HTML・SVGグラフを作る。
- `/data/experiment_data/scripts/run_sdk551_body_model_comparison_20261022.py` と共有パッケージの `sdk_tools/` に今回のコードを新規配置。過去に配布したランナーやSDK出力は上書きしていない。

構文と引数表示を確認し、過去の実SDK出力を再集計した。ユニットテストは依頼されておらず実行していない。新モデルをGPUで実行したり、合格・優劣を検証したりした段階ではない。録画設定・原本・運用中の校正JSONを変更していない。共有先での推論後は、その結果を使って003のポール下での確認も続ける。
