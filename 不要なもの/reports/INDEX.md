# 解析レポート一覧

詳細レポートは監査記録として保持する。必要な案件だけ開く。

| 文書・フォルダー | 内容 |
| --- | --- |
| `run_order_audit.md` | 試行計画・録画メタデータの監査。 |
| `review_20261020/REVIEW.md` | 追跡・融合・校正コードのレビュー、変更内容、実録画と合成データの検証限界。 |
| `review_20261020/REPRODUCE.md` | 上記レビューの再現手順、参照ラベル仕様、同期・校正の注意。 |
| `review_20261020/ARUCO_20261019_NEXT_TASKS.md` | ArUco校正録画の失敗原因、追加取得条件、完了基準。計画時点の記録。 |
| `pole_calibration_results/` | 7地点のポールによる暫定ZED/LiDAR投影、深度整合性、限界。最終校正値ではない。 |
| `20261021_group002_calibration_inputs.md` | 10/21グループ2校正の入力・ユーザー申告・未確認条件。 |
| `20261021_group002_recording_002_pole_check/README.md` | 録画002のZEDレビュー出力、時刻・画像・深度の確認。 |
| `20261021_group002_pole_calibration/` | 録画002ポール候補、処理コード、入力キャッシュと投影画像。運用設定には未適用。 |
| `20261021_group002_recording_003_walk_assessment/README.md` | 録画003の3〜15 m往復。確定検出の欠測、候補を含む経路の整合性、上面図・映像の再生、絶対精度の限界。 |
| `20261021_group002_recording_003_skeleton_replay/verified_20261022/README.md` | 録画003の全899フレームの骨格出力確認。折返し後の骨格形状の崩れ、信頼度の限界、骨格・廊下・映像の3D再生。 |
| `20261021_group002_recording_003_skeleton_replay/display_fix_20261022/README.md` | 最新の再生画面。骨格の拡大、床と高さZの目盛り、上面図との区別、復路の不整合骨格を紫で表示。 |
| `20261021_group002_recording_003_skeleton_replay/approach_check_20261022/README.md` | 本番の接近動作を模したSDK新規追跡の条件・手順。約15 m・13 m開始の実行結果と上面図の直接選択は `RESULT_20261022.md` と共有再生画面を参照。 |
| `20261021_group002_recording_003_skeleton_replay/approach_check_20261022/ZED_LIDAR_COMPARISON_20261022.md` | 録画003の奥から接近する区間だけでZED SDK骨格とLiDAR ID 60の経路を比較。距離帯別の相対差、候補状態、同期・校正の制限。 |
| `door_calibration_20261013/README.md` | ドア凹部ICP校正の試行。検証で悪化したため候補を棄却。 |
| `independent_walk_20261013/README.md` | 校正歩行とは独立した1人往復の確認と限界。 |
| `tracking/20261013_group001_recording_001_analysis.md` | 録画001のLiDAR追跡再解析、距離、ZED比較、部分校正。 |
| `recordings/*/RESULTS.md` | 2026-10-05のZED圧縮・Livox容量・保存時間の測定記録。 |
| `calibration_gui/README.md` | 校正GUIの起動、入力形式、座標系、開発確認。 |
| `docs/trajectory_human_review.md` | 追跡の再生、人手確認項目、独立真値による評価手順。 |
