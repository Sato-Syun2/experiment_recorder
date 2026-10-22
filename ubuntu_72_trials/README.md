# 追い越し実験：Ubuntu録画用 72試行プラン

本プランは実際に印刷した `追い越し実験_配布用_A4横_全シート統一(1).xlsx`
および元の「実験者用条件表」を照合して生成しました。紙に印刷された各セットの
**試行01〜18の順番を一切変更していません**。

## 構造

- 各グループ：18試行×4セット＝72試行。
- 最初の54試行（セット1〜3）は印刷ガイド記載のA/B/C1またはC2。
- 試行55〜72は **未使用のC1またはC2**。CSVの`optional=1`で識別。
- 18グループすべてで「A, B, C1, C2」を各1セット実施。
- 1グループ：追い越し64件、単独8件。
- 各被験者：追い越す32件、追い越される32件、単独4件。
- E01〜E18はそれぞれ4回出現。追い越しE01〜E16は各条件で追越者P1/P2が2回ずつ。
- 用紙と録画の照合：`set_id`（A/B/C1/C2）と`set_trial_no`（01〜18）で行う。

## 同梱ファイル

- `all_trials.csv`：18グループ×72試行=1296行（全グループをまとめて処理する場合）
- `groups/G01.csv` ～ `groups/G18.csv`：グループ別72行（録画時はこれが簡単）
- `groups/G01.json` ～ `groups/G18.json`：同内容のJSON（Python録画プログラム向け）
- `group_order.csv`：グループごとの4セット順。印刷ガイドどおり。
- `set_templates.csv`：用紙A/B/C1/C2での1〜18の対応表と教示
- `condition_master.csv`：E01〜E18の条件定義
- `manifest.json`：件数と符号化規則、印刷元ファイルのSHA256
- `validate_plan.py`：標準ライブラリのみで件数・重複・役割を検査
- `CODEX_INTEGRATION.md`：録画ソフト修正仕様

## Ubuntuでの使用

```bash
mkdir -p ~/experiment_plan_72
unzip ubuntu_72_trials.zip -d ~/experiment_plan_72
python3 ~/experiment_plan_72/validate_plan.py

# G07を録画する場合
head -n 3 ~/experiment_plan_72/groups/G07.csv
```

### CSV/JSONの重要な列

| フィールド | 意味 |
| --- | --- |
| `group_id` | G01〜G18 |
| `trial_no` | グループ内の**通し番号**1〜72 |
| `trial_id` | グループ内で重複しない不変ID（例：`G01_T055`） |
| `set_no` | 1〜4 |
| `set_id` | A、B、C1、C2（印刷用紙の識別子） |
| `set_trial_no` | 印刷用紙内の試行番号1〜18 |
| `condition_id` | E01〜E18（反復で同じIDが使われる） |
| `trial_type` | `overtaking`か`solo` |
| `width` | `wide`、`narrow` |
| `avoidance` | `avoid`、`no_avoid`、空（単独） |
| `speed` | `normal`、`slow` |
| `pass_side` | `right`、`left`、空（単独） |
| `overtaker`/`overtaken` | `P1`、`P2`、空（単独） |
| `solo_person` | `P1`、`P2`、空（追い越し） |
| `person1_instruction`/`person2_instruction` | 印刷用紙と一致する教示 |
| `width_change_before` | 直前の試行から幅変更が必要なら`1` |
| `optional` | **試行55〜72**なら`1`（追加セット） |

CSVは **UTF-8 BOM付き** です。Pythonは `encoding="utf-8-sig"` で読み込んでください。
JSONはUTF-8です。

## 録画プログラム側で守ること

1. 既存の録画処理（ZED SVO2 / LiDAR bagなど）は基本的に変更不要。
   変更点は「スケジュールの読み込み、表示、終了条件、録画名、結果記録」。
2. `group_id`を選択してCSVまたはJSONの72行を読み込み、上記の順番に実行。
   **録画時にランダム化しない**。紙の順番と一致させる。
3. 通し番号は`trial_no`、用紙の照合は `set_id`と`set_trial_no`。
   セットの切り替わりや `width_change_before=1` のとき確認を表示。
4. **録画後のやり直し**では `trial_id` は固定し、`take_no` を増やす。
   NGデータを上書き・消去しない。
5. 例：`G01_T055_S04_C2_E01_take01_YYYYMMDD_HHMMSS.svo2`
6. 録画成否・日時・ファイルパスなどの実績はCSV原本に書かず、
   `recording_log.jsonl`や`progress.json`など別ファイルに保存する。
7. 54試行時点で追加を実施しない場合は**実験中断/追加未実施**を明確に区別する。
   55〜72を失敗した試行と混同しない。
8. 72は**成功した固有試行数**。再録でTake数が72を超えても問題ない。
9. 再起動した場合、ログで `COMPLETE` を判定して続きから再開する。
10. 条件実施順は固定しているので、録画ソフトからの自動並べ替えは禁止。

## 注意

- `trial_id`と`condition_id`は異なります。E01は各グループ4回出現します。
- 「72条件」ではなく「18種類の条件を4反復した72**試行**」です。
- 実験後に録画データと紙回答を結合するキーは
  `group_id`+`set_id`+`set_trial_no`（あるいは`trial_id`）です。
- 計画に修正が必要なら、**紙と録画プランを両方更新**してバージョンを残してください。
