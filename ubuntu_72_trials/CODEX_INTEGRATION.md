# Codexへの依頼：録画ソフトを72試行に対応

現在の録画プログラムのコードを読み、録画処理自体を壊さずに、同梱の `groups/Gxx.csv`
または `groups/Gxx.json` に基づく録画順管理へ変更してください。

## 固定条件
- 試行スケジュールはCSV/JSONをそのまま採用し、再シャッフルしない。
- グループはG01〜G18、グループ内通し試行は1〜72。
- セット1〜3（1〜54）が基本。セット4（55〜72）は`optional=1`の追加試行。
- `set_id` + `set_trial_no` が印刷した被験者用紙と一致する。
- condition_id(E01〜E18)は反復するので、それだけをファイル名にしてはならない。

## 必要な変更の判断と実装
1. `54`、`3 sets`、`ABC only`等のハードコードを探し、可変長スケジュールに変更。
2. 起動時に`--group G01 --plan /path/to/ubuntu_72_trials/groups/G01.csv`などで読込
   （CLIの具体名は既存仕様に合わせてよい）。
3. 前回進捗の復元、次の試行、幅変更、セット移行、被験者の役割・教示を表示。
4. `trial_id`を固定識別子に、録画ファイルはtake番号で別名保存。
5. START/STOP/COMPLETE/INVALID/RETRY/SKIPを区別し、原本CSVを変更しない。
   特に `RETRY` は同じ`trial_id`に新しいtake番号を付ける。
6. START直前に録画対象と保存パスを表示し、録画後に生成ファイル存在・非空を確認。
7. 54試行で第4セットを省略する操作も用意。ただし`optional`試行はPENDING_OPTIONALとして記録。
8. 録画終了時にmetadata.json（group_id,trial_id,set_no,set_id,set_trial_no,
   condition_id,trial_type,width,avoidance,speed,pass_side,overtaker,overtaken,solo_person,
   take_no,start_time,end_time,status,file_paths）を保存。
9. 既存のZED/LiDAR録画処理は可能ならそのまま使う。ファイルの上書き防止を優先。
10. `validate_plan.py` と自動テストで、G01/G18の順序、各グループ72件、各条件4回、
    被験者別32/32、単独4回、再録と再起動復帰を確認する。

修正前に現状のモジュール構成とハードコード箇所を特定し、修正差分、起動方法、
バックアップ方針、検証結果を報告してください。
