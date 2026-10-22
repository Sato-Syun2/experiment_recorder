#!/usr/bin/env python3
"""Excelから録画用の共通順序データを作成・検証する。"""

from __future__ import annotations

import hashlib
import json
import os
import posixpath
import re
import zipfile
from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT_DIR = PROJECT_ROOT / "inputs"
DEFAULT_PLAN_PATH = PROJECT_ROOT / "run_order.json"
DEFAULT_AUDIT_PATH = PROJECT_ROOT / "reports" / "run_order_audit.md"
PLAN_WORKBOOK_NAME = "追い越し実験_完成版_18グループ.xlsx"
PRINT_WORKBOOK_NAME = "追い越し実験_印刷用_6設問_簡潔教示.xlsx"
PLAN_VERSION = 1
XML_NS = {
    "main": "http://schemas.openxmlformats.org/spreadsheetml/2006/main",
    "office_rel": "http://schemas.openxmlformats.org/officeDocument/2006/relationships",
    "package_rel": "http://schemas.openxmlformats.org/package/2006/relationships",
}
PAPER_SETS = ("A", "B", "C1", "C2")
GROUP_COUNT = 18
BASIC_TRIAL_COUNT = 54
ADDITIONAL_TRIAL_COUNT = 18


class PlanError(RuntimeError):
    """読み込んだ実験計画または配布用紙に矛盾がある。"""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_plan_hash(plan: dict[str, Any]) -> str:
    """plan_sha256自身を除いたJSON内容から再現可能な計画ハッシュを作る。"""

    payload = {key: value for key, value in plan.items() if key != "plan_sha256"}
    serialized = json.dumps(
        payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(serialized).hexdigest()


def _column_number(cell_reference: str) -> int:
    match = re.match(r"([A-Z]+)", cell_reference)
    if not match:
        raise PlanError(f"Excelのセル番地を解釈できません: {cell_reference}")
    number = 0
    for character in match.group(1):
        number = number * 26 + ord(character) - ord("A") + 1
    return number


def _read_xlsx(path: Path) -> dict[str, list[dict[int, str]]]:
    """外部ライブラリなしでxlsxのシート名・セル値を読み込む。"""

    path = Path(path)
    try:
        archive = zipfile.ZipFile(path)
    except (OSError, zipfile.BadZipFile) as exc:
        raise PlanError(f"Excelファイルを開けません: {path}: {exc}") from exc

    with archive:
        names = set(archive.namelist())
        try:
            workbook = ET.fromstring(archive.read("xl/workbook.xml"))
            relationships = ET.fromstring(archive.read("xl/_rels/workbook.xml.rels"))
        except (KeyError, ET.ParseError) as exc:
            raise PlanError(f"Excelのブック構造を読めません: {path}") from exc

        relationship_targets = {
            relation.attrib["Id"]: relation.attrib["Target"]
            for relation in relationships.findall("package_rel:Relationship", XML_NS)
        }
        shared_strings: list[str] = []
        if "xl/sharedStrings.xml" in names:
            try:
                root = ET.fromstring(archive.read("xl/sharedStrings.xml"))
            except ET.ParseError as exc:
                raise PlanError(f"共有文字列を読めません: {path}") from exc
            shared_strings = [
                "".join(text.text or "" for text in item.findall(".//main:t", XML_NS))
                for item in root.findall("main:si", XML_NS)
            ]

        sheets: dict[str, list[dict[int, str]]] = {}
        for sheet in workbook.findall("main:sheets/main:sheet", XML_NS):
            sheet_name = sheet.attrib["name"]
            relationship_id = sheet.attrib[f"{{{XML_NS['office_rel']}}}id"]
            target = relationship_targets.get(relationship_id)
            if not target:
                raise PlanError(f"シート {sheet_name} のXMLが見つかりません: {path}")
            if target.startswith("/"):
                sheet_path = target.lstrip("/")
            else:
                sheet_path = posixpath.normpath(posixpath.join("xl", target))
            try:
                root = ET.fromstring(archive.read(sheet_path))
            except (KeyError, ET.ParseError) as exc:
                raise PlanError(f"シート {sheet_name} を読めません: {path} ({sheet_path})") from exc

            rows: list[dict[int, str]] = []
            for row in root.findall("main:sheetData/main:row", XML_NS):
                cells: dict[int, str] = {}
                for cell in row.findall("main:c", XML_NS):
                    value = cell.find("main:v", XML_NS)
                    cell_type = cell.attrib.get("t")
                    if cell_type == "inlineStr":
                        content = "".join(
                            text.text or "" for text in cell.findall(".//main:t", XML_NS)
                        )
                    elif value is None:
                        content = ""
                    elif cell_type == "s":
                        try:
                            content = shared_strings[int(value.text or "0")]
                        except (ValueError, IndexError) as exc:
                            raise PlanError(f"共有文字列の参照が壊れています: {path}") from exc
                    else:
                        content = value.text or ""
                    if content != "":
                        cells[_column_number(cell.attrib["r"])] = content
                rows.append(cells)
            sheets[sheet_name] = rows
        return sheets


def _table(
    workbook: dict[str, list[dict[int, str]]],
    sheet_name: str,
    required_headers: tuple[str, ...],
    *,
    workbook_name: str,
) -> list[dict[str, str]]:
    if sheet_name not in workbook:
        raise PlanError(f"{workbook_name}: 必要なシートがありません: {sheet_name}")
    rows = workbook[sheet_name]
    header_row_index = None
    header_columns: dict[str, int] = {}
    for row_index, row in enumerate(rows):
        candidate = {value.strip(): column for column, value in row.items()}
        if all(header in candidate for header in required_headers):
            header_row_index = row_index
            header_columns = candidate
            break
    if header_row_index is None:
        raise PlanError(
            f"{workbook_name} / {sheet_name}: 列名を確認できません。"
            f"必要な見出し: {', '.join(required_headers)}"
        )

    result: list[dict[str, str]] = []
    for row in rows[header_row_index + 1 :]:
        record = {
            header: (
                row.get(header_columns[header], "")
                if header == "今日の教示"
                else row.get(header_columns[header], "").strip()
            )
            for header in required_headers
        }
        if any(record.values()):
            result.append(record)
    return result


def _integer(value: str, location: str) -> int:
    try:
        number = float(value)
        if not number.is_integer():
            raise ValueError
        return int(number)
    except (TypeError, ValueError) as exc:
        raise PlanError(f"{location}: 整数ではありません: {value!r}") from exc


def _optional(value: str) -> str | None:
    return None if value.strip() in {"", "—", "-"} else value.strip()


def _require_exact(
    differences: list[str], location: str, left_name: str, left: Any, right_name: str, right: Any
) -> None:
    if left != right:
        differences.append(f"{location}: {left_name}={left!r} / {right_name}={right!r}")


def _source_records(
    plan_book: dict[str, list[dict[int, str]]],
    print_book: dict[str, list[dict[int, str]]],
    differences: list[str],
) -> tuple[Any, ...]:
    master_rows = _table(
        plan_book,
        "02_条件マスター",
        ("条件ID", "区分", "通路幅", "回避", "前方者速度", "追越方向", "単独対象者"),
        workbook_name=PLAN_WORKBOOK_NAME,
    )
    conditions: dict[str, dict[str, str]] = {}
    for row in master_rows:
        condition_id = row["条件ID"]
        if not condition_id:
            continue
        if condition_id in conditions:
            differences.append(f"条件マスター: 条件ID {condition_id} が重複しています")
        conditions[condition_id] = row
    expected_ids = {f"E{number:02d}" for number in range(1, 19)}
    _require_exact(
        differences,
        "02_条件マスター",
        "条件ID集合",
        set(conditions),
        "期待値",
        expected_ids,
    )

    experimenter_orders: dict[str, list[dict[str, str]]] = {}
    for paper_set in PAPER_SETS:
        experimenter_orders[paper_set] = _table(
            plan_book,
            f"{paper_set}_実験者",
            (
                "順番",
                "条件ID",
                "区分",
                "通路幅",
                "回避",
                "速度",
                "追越方向",
                "追越者",
                "被追越者",
                "単独対象",
            ),
            workbook_name=PLAN_WORKBOOK_NAME,
        )
        experimenter_orders[paper_set] = [
            row for row in experimenter_orders[paper_set] if row["順番"]
        ]
        if len(experimenter_orders[paper_set]) != 18:
            differences.append(
                f"{paper_set}_実験者: 18行の計画が必要ですが"
                f"{len(experimenter_orders[paper_set])}行です"
            )
        for index, row in enumerate(experimenter_orders[paper_set], start=1):
            location = f"{paper_set}_実験者 試行{index:02d}"
            _require_exact(
                differences,
                location,
                "順番",
                _integer(row["順番"], location),
                "期待値",
                index,
            )
            condition = conditions.get(row["条件ID"])
            if condition is None:
                differences.append(f"{location}: 条件ID {row['条件ID']} が条件マスターにありません")
                continue
            for source_header, master_header in (
                ("区分", "区分"),
                ("通路幅", "通路幅"),
                ("回避", "回避"),
                ("速度", "前方者速度"),
                ("追越方向", "追越方向"),
                ("単独対象", "単独対象者"),
            ):
                # 単独条件ではセット表の幅・速度が実施環境の値を示す。
                if condition["区分"] == "単独歩行" and source_header in {
                    "通路幅",
                    "回避",
                    "速度",
                    "追越方向",
                }:
                    continue
                _require_exact(
                    differences,
                    location,
                    source_header,
                    _optional(row[source_header]),
                    f"条件マスター {master_header}",
                    _optional(condition[master_header]),
                )

    # 印刷シートは行番号・役割・教示を列見出しから読み、文面は原文のまま保持する。
    print_sheets: dict[str, dict[int, dict[int, dict[str, str]]]] = {}
    for paper_set in PAPER_SETS:
        participant_sheets: dict[int, dict[int, dict[str, str]]] = {}
        for subject_number in (1, 2):
            sheet_name = f"{paper_set}_被験者{subject_number}"
            rows = _table(
                print_book,
                sheet_name,
                ("試行", "役割", "今日の教示"),
                workbook_name=PRINT_WORKBOOK_NAME,
            )
            numbered: dict[int, dict[str, str]] = {}
            for row in rows:
                if not row["試行"].isdigit():
                    continue
                paper_trial = int(row["試行"])
                if paper_trial in numbered:
                    differences.append(
                        f"{sheet_name}: 用紙内試行番号 {paper_trial} が重複しています"
                    )
                numbered[paper_trial] = row
            if set(numbered) != set(range(1, 19)):
                differences.append(
                    f"{sheet_name}: 用紙内番号は1〜18が必要です。" f"実際={sorted(numbered)}"
                )
            participant_sheets[subject_number] = numbered

            title_values = [
                value.strip() for row in print_book[sheet_name][:4] for value in row.values()
            ]
            if not any(value == f"セット {paper_set}" for value in title_values):
                differences.append(
                    f"{sheet_name}: シート内のセット名が「セット {paper_set}」ではありません"
                )
        print_sheets[paper_set] = participant_sheets

    return master_rows, experimenter_orders, conditions, print_sheets


def _semantic_instruction_check(
    differences: list[str],
    location: str,
    role: str,
    instruction: str,
    condition: dict[str, str],
) -> None:
    """印刷文の内容が条件と一致するか確認する。文面自体は書き換えない。"""

    if role == "追い越す":
        side = "右" if "右側" in instruction else "左" if "左側" in instruction else None
        _require_exact(
            differences,
            location,
            "教示から読み取った追越方向",
            side,
            "条件マスターの追越方向",
            _optional(condition["追越方向"]),
        )
    elif role == "追い越される":
        speed = (
            "遅い" if "ゆっくり" in instruction else "通常" if "普段どおり" in instruction else None
        )
        _require_exact(
            differences,
            location,
            "教示から読み取った前方者速度",
            speed,
            "条件マスターの前方者速度",
            _optional(condition["前方者速度"]),
        )
        if any(term in instruction for term in ("道を譲る", "側方によけ", "避けて")):
            avoidance = "避ける"
        elif any(term in instruction for term in ("真っすぐ", "進路を変えない")):
            avoidance = "避けない"
        else:
            avoidance = None
        _require_exact(
            differences,
            location,
            "教示から読み取った回避",
            avoidance,
            "条件マスターの回避",
            _optional(condition["回避"]),
        )


def _normalize_plan_fields(
    row: dict[str, str],
    *,
    group_name: str,
    global_trial: int,
    set_number: int,
    paper_set: str,
    paper_trial: int,
    condition: dict[str, str],
    print_sheets: dict[str, dict[int, dict[int, dict[str, str]]]],
    differences: list[str],
) -> dict[str, Any]:
    is_additional = global_trial > BASIC_TRIAL_COUNT
    condition_id = row["条件ID"]
    location = f"{group_name} 全体試行{global_trial:02d} ({paper_set} {paper_trial:02d})"

    plan_category = _optional(row["区分"])
    master_category = _optional(condition["区分"])
    _require_exact(
        differences, location, "グループ表の区分", plan_category, "条件マスター", master_category
    )
    # 追越条件では各要因が条件マスターと一致する。単独条件の幅・通常速度は
    # その試行時の実施環境を示すので、条件マスターの「—」とは比較しない。
    if master_category == "追い越し":
        for plan_header, master_header in (
            ("通路幅", "通路幅"),
            ("回避", "回避"),
            ("前方速度", "前方者速度"),
            ("追越側", "追越方向"),
        ):
            _require_exact(
                differences,
                location,
                f"グループ表 {plan_header}",
                _optional(row[plan_header]),
                f"条件マスター {master_header}",
                _optional(condition[master_header]),
            )
    else:
        _require_exact(
            differences,
            location,
            "グループ表の単独対象",
            _optional(row["単独対象"]),
            "条件マスターの単独対象者",
            _optional(condition["単独対象者"]),
        )

    subject_data: dict[str, dict[str, Any]] = {}
    for subject_number in (1, 2):
        print_row = print_sheets[paper_set][subject_number].get(paper_trial)
        if print_row is None:
            differences.append(f"{location}: 被験者{subject_number}の配布試行がありません")
            continue
        role = print_row["役割"]
        instruction = print_row["今日の教示"]
        subject_data[f"subject_{subject_number}"] = {
            "role": role,
            "instruction": instruction,
        }
        if not instruction:
            differences.append(f"{location}: 被験者{subject_number}の教示文が空欄です")

    if master_category == "単独歩行":
        solo_target = _optional(row["単独対象"])
        expected_roles = {
            "被験者1": "単独歩行" if solo_target == "被験者1" else "待機",
            "被験者2": "単独歩行" if solo_target == "被験者2" else "待機",
        }
    else:
        overtaker = _optional(row["追越者"])
        overtaken = _optional(row["被追越者"])
        expected_roles = {
            overtaker: "追い越す",
            overtaken: "追い越される",
        }
        solo_target = None

    for subject_number in (1, 2):
        subject_label = f"被験者{subject_number}"
        subject_key = f"subject_{subject_number}"
        subject = subject_data.get(subject_key)
        if not subject:
            continue
        expected_role = expected_roles.get(subject_label)
        _require_exact(
            differences,
            f"{location} {subject_label}",
            "配布プリントの役割",
            subject["role"],
            "グループ計画から期待する役割",
            expected_role,
        )
        _semantic_instruction_check(
            differences,
            f"{location} {subject_label}",
            subject["role"],
            subject["instruction"],
            condition,
        )
        if subject["role"] == "単独歩行" and "1人で歩いて" not in subject["instruction"]:
            differences.append(
                f"{location} {subject_label}: 単独歩行の教示文を確認してください: "
                f"{subject['instruction']!r}"
            )
        if subject["role"] == "待機" and "待機" not in subject["instruction"]:
            differences.append(
                f"{location} {subject_label}: 待機の教示文を確認してください: "
                f"{subject['instruction']!r}"
            )

    return {
        "group": group_name,
        "global_trial_number": global_trial,
        "set_number": set_number,
        "paper_name": paper_set,
        "paper_trial_number": paper_trial,
        "condition_id": condition_id,
        "trial_type": "additional" if is_additional else "basic",
        "trial_type_label": "追加試行（任意）" if is_additional else "基本試行",
        "condition_category": plan_category,
        "corridor_width": _optional(row["通路幅"]),
        "avoidance": _optional(row["回避"]),
        "leading_walker_speed": _optional(row["前方速度"]),
        "overtaking_direction": _optional(row["追越側"]),
        "overtaker": _optional(row["追越者"]),
        "overtaken_person": _optional(row["被追越者"]),
        "solo_target": solo_target,
        "subject_1": subject_data.get("subject_1"),
        "subject_2": subject_data.get("subject_2"),
    }


def _markdown_cell(value: Any) -> str:
    return str(value if value is not None else "—").replace("|", "\\|").replace("\n", "<br>")


def _build_audit_report(
    plan: dict[str, Any],
    schedules: list[dict[str, Any]],
) -> str:
    sources = plan["source_workbooks"]
    lines = [
        "# 実験計画・配布プリント照合",
        "",
        "この表は確認済みExcelから生成した `run_order.json` と同じ計画データを使っています。",
        "各グループは基本54試行と第4セット18試行の合計72試行です。録画ソフトは全72試行を通常の進行対象にします。",
        "Excel由来の基本・追加区分は照合用に保持しています。54試行で終了する場合はQで中断し、未完了の第4セットは再開対象として残ります。",
        "",
        f"- 計画データ版: {plan['plan_version']}",
        f"- 計画データSHA-256: `{plan['plan_sha256']}`",
        f"- 完成版Excel: `{sources['completed_plan']['filename']}` / SHA-256 `{sources['completed_plan']['sha256']}`",
        f"- 印刷用Excel: `{sources['printable_booklet']['filename']}` / SHA-256 `{sources['printable_booklet']['sha256']}`",
        "- 照合結果: 不一致なし。条件IDは完成版Excel由来で、配布プリントに幅・条件IDがないことは不一致にしていません。",
        "",
        "## グループ別セット順",
        "",
        "| グループ | 第1セット（01–18） | 第2セット（19–36） | 第3セット（37–54） | 第4セット（55–72） |",
        "|---|---|---|---|---|",
    ]
    for schedule in schedules:
        lines.append(
            "| {group} | {set_1} | {set_2} | {set_3} | {optional_set} |".format(
                group=schedule["group"],
                set_1=schedule["basic_sets"][0],
                set_2=schedule["basic_sets"][1],
                set_3=schedule["basic_sets"][2],
                optional_set=schedule["optional_set"],
            )
        )

    lines.extend(
        [
            "",
            "## 配布プリントと録画順（セットごと18試行）",
            "",
            "各グループ表の用紙内番号1〜18、セット別実験者表の条件ID、被験者1・2用プリントを同じ行で照合しています。教示欄は印刷用Excelの文面をそのまま掲載しています。",
            "",
            "| セット | 用紙内番号 | 条件ID | 被験者1の役割・教示 | 被験者2の役割・教示 | 照合 |",
            "|---|---:|---|---|---|---|",
        ]
    )
    for paper_set in PAPER_SETS:
        set_trials = plan["set_templates"][paper_set]
        for trial in set_trials:
            subject_1 = trial["subject_1"]
            subject_2 = trial["subject_2"]
            lines.append(
                "| {paper} | {index:02d} | {condition} | {role1}：{instruction1} | {role2}：{instruction2} | 一致 |".format(
                    paper=paper_set,
                    index=trial["paper_trial_number"],
                    condition=trial["condition_id"],
                    role1=_markdown_cell(subject_1["role"]),
                    instruction1=_markdown_cell(subject_1["instruction"]),
                    role2=_markdown_cell(subject_2["role"]),
                    instruction2=_markdown_cell(subject_2["instruction"]),
                )
            )
    lines.append("")
    return "\n".join(lines)


def build_run_order(
    input_dir: Path = DEFAULT_INPUT_DIR,
    *,
    plan_path: Path = DEFAULT_PLAN_PATH,
    audit_path: Path = DEFAULT_AUDIT_PATH,
) -> dict[str, Any]:
    """両Excelを全台帳・条件・配布教示と照合し、計画JSONと監査表を作成する。"""

    input_dir = Path(input_dir).resolve()
    plan_workbook_path = input_dir / PLAN_WORKBOOK_NAME
    print_workbook_path = input_dir / PRINT_WORKBOOK_NAME
    missing = [path for path in (plan_workbook_path, print_workbook_path) if not path.is_file()]
    if missing:
        missing_names = ", ".join(str(path) for path in missing)
        raise PlanError(f"必要なExcelがありません: {missing_names}")

    source_hashes_before = {
        "completed_plan": sha256_file(plan_workbook_path),
        "printable_booklet": sha256_file(print_workbook_path),
    }
    plan_book = _read_xlsx(plan_workbook_path)
    print_book = _read_xlsx(print_workbook_path)
    differences: list[str] = []
    _, experimenter_orders, conditions, print_sheets = _source_records(
        plan_book, print_book, differences
    )

    order_rows = _table(
        plan_book,
        "01_グループ順",
        (
            "組",
            "ABC順",
            "第1セット",
            "第2セット",
            "第3セット",
            "C型",
            "幅変更",
            "第4セット（任意）",
            "試行数",
            "追越者1",
            "追越者2",
            "判定",
            "追加後の計画試行数",
            "追越者1（計画）",
            "追越者2（計画）",
            "追加後検算",
        ),
        workbook_name=PLAN_WORKBOOK_NAME,
    )
    group_schedules: dict[str, dict[str, Any]] = {}
    for row in order_rows:
        group_name = row["組"]
        if not re.fullmatch(r"G\d{2}", group_name):
            continue
        basic_sets = [row[f"第{number}セット"] for number in (1, 2, 3)]
        optional_set = row["第4セット（任意）"]
        expected_order = [item.strip() for item in row["ABC順"].split("→")]
        normalized_order = ["C" if item in {"C1", "C2", "C"} else item for item in basic_sets]
        _require_exact(
            differences,
            f"01_グループ順 {group_name}",
            "ABC順から読み取ったセット順",
            expected_order,
            "第1〜3セット欄",
            normalized_order,
        )
        expected_c_type = next((item for item in basic_sets if item in {"C1", "C2"}), None)
        other_c_type = "C2" if expected_c_type == "C1" else "C1"
        _require_exact(
            differences,
            f"01_グループ順 {group_name}",
            "C型",
            row["C型"],
            "基本計画で実施するCセット",
            expected_c_type,
        )
        _require_exact(
            differences,
            f"01_グループ順 {group_name}",
            "第4セット（任意）",
            optional_set,
            "基本計画で未実施のCセット",
            other_c_type,
        )
        _require_exact(
            differences,
            f"01_グループ順 {group_name}",
            "試行数",
            _integer(row["試行数"], f"01_グループ順 {group_name}"),
            "基本試行数",
            BASIC_TRIAL_COUNT,
        )
        group_schedules[group_name] = {
            "group": group_name,
            "group_number": int(group_name[1:]),
            "basic_sets": basic_sets,
            "optional_set": optional_set,
            "expected_width_changes": _integer(row["幅変更"], f"01_グループ順 {group_name} 幅変更"),
            "expected_basic_overtaker_1": _integer(
                row["追越者1"], f"01_グループ順 {group_name} 追越者1"
            ),
            "expected_basic_overtaker_2": _integer(
                row["追越者2"], f"01_グループ順 {group_name} 追越者2"
            ),
            "basic_status": row["判定"],
            "expected_total_after_additional": _integer(
                row["追加後の計画試行数"], f"01_グループ順 {group_name} 追加後の計画試行数"
            ),
            "expected_total_overtaker_1": _integer(
                row["追越者1（計画）"], f"01_グループ順 {group_name} 追越者1（計画）"
            ),
            "expected_total_overtaker_2": _integer(
                row["追越者2（計画）"], f"01_グループ順 {group_name} 追越者2（計画）"
            ),
            "additional_status": row["追加後検算"],
        }
    _require_exact(
        differences,
        "01_グループ順",
        "グループ数",
        len(group_schedules),
        "期待値",
        GROUP_COUNT,
    )

    # グループ別表を一次資料として読むが、972件の全体台帳と324件の追加台帳、
    # グループ順表、条件マスター、印刷表の全てと比較してから確定する。
    base_master_rows = _table(
        plan_book,
        "03_全972試行",
        (
            "グループ",
            "全体順",
            "第何セット",
            "用紙",
            "条件ID",
            "区分",
            "通路幅",
            "回避",
            "前方速度",
            "追越側",
            "追越者",
            "被追越者",
            "単独対象",
        ),
        workbook_name=PLAN_WORKBOOK_NAME,
    )
    additional_ledger_rows = _table(
        plan_book,
        "06_追加324試行",
        (
            "グループ",
            "全体順",
            "第何セット",
            "用紙",
            "条件ID",
            "区分",
            "通路幅",
            "回避",
            "前方速度",
            "追越側",
            "追越者",
            "被追越者",
            "単独対象",
        ),
        workbook_name=PLAN_WORKBOOK_NAME,
    )

    def common_record(row: dict[str, str]) -> dict[str, Any]:
        return {
            "group": row["グループ"],
            "global_trial_number": _integer(row["全体順"], "全体順"),
            "set_number": _integer(row["第何セット"], "第何セット"),
            "paper_name": row["用紙"],
            "condition_id": row["条件ID"],
            "condition_category": _optional(row["区分"]),
            "corridor_width": _optional(row["通路幅"]),
            "avoidance": _optional(row["回避"]),
            "leading_walker_speed": _optional(row["前方速度"]),
            "overtaking_direction": _optional(row["追越側"]),
            "overtaker": _optional(row["追越者"]),
            "overtaken_person": _optional(row["被追越者"]),
            "solo_target": _optional(row["単独対象"]),
        }

    base_master = {
        (record["group"], record["global_trial_number"]): record
        for record in (common_record(row) for row in base_master_rows if row["全体順"])
    }
    additional_ledger = {
        (record["group"], record["global_trial_number"]): record
        for record in (common_record(row) for row in additional_ledger_rows if row["全体順"])
    }
    _require_exact(
        differences,
        "03_全972試行",
        "行数",
        len(base_master),
        "期待値",
        GROUP_COUNT * BASIC_TRIAL_COUNT,
    )
    _require_exact(
        differences,
        "06_追加324試行",
        "行数",
        len(additional_ledger),
        "期待値",
        GROUP_COUNT * ADDITIONAL_TRIAL_COUNT,
    )

    full_plan_records: list[dict[str, Any]] = []
    group_templates: dict[str, list[dict[str, Any]]] = {}
    for group_name, schedule in sorted(group_schedules.items()):
        if group_name not in plan_book:
            differences.append(f"グループ表がありません: {group_name}")
            continue
        group_rows = _table(
            plan_book,
            group_name,
            (
                "全体順",
                "第何セット",
                "用紙",
                "用紙内順",
                "条件ID",
                "区分",
                "通路幅",
                "回避",
                "前方速度",
                "追越側",
                "追越者",
                "被追越者",
                "単独対象",
            ),
            workbook_name=PLAN_WORKBOOK_NAME,
        )
        group_rows = [row for row in group_rows if row["全体順"]]
        row_numbers = [_integer(row["全体順"], f"{group_name} 全体順") for row in group_rows]
        _require_exact(
            differences,
            f"{group_name}",
            "全体試行番号",
            row_numbers,
            "連続する計画番号",
            list(range(1, BASIC_TRIAL_COUNT + ADDITIONAL_TRIAL_COUNT + 1)),
        )
        group_entries: list[dict[str, Any]] = []
        for row in group_rows:
            global_trial = _integer(row["全体順"], f"{group_name} 全体順")
            set_number = _integer(
                row["第何セット"], f"{group_name} 全体試行{global_trial} セット番号"
            )
            paper_set = row["用紙"]
            paper_trial = _integer(
                row["用紙内順"], f"{group_name} 全体試行{global_trial} 用紙内番号"
            )
            is_additional = global_trial > BASIC_TRIAL_COUNT
            expected_set_number = (global_trial - 1) // 18 + 1
            expected_set = (
                schedule["basic_sets"][expected_set_number - 1]
                if expected_set_number <= 3
                else schedule["optional_set"]
            )
            _require_exact(
                differences,
                f"{group_name} 全体試行{global_trial}",
                "セット番号",
                set_number,
                "順番から期待するセット番号",
                expected_set_number,
            )
            _require_exact(
                differences,
                f"{group_name} 全体試行{global_trial}",
                "用紙名",
                paper_set,
                "グループ順から期待する用紙",
                expected_set,
            )
            _require_exact(
                differences,
                f"{group_name} 全体試行{global_trial}",
                "用紙内番号",
                paper_trial,
                "順番から期待する番号",
                (global_trial - 1) % 18 + 1,
            )

            condition = conditions.get(row["条件ID"])
            if condition is None:
                differences.append(
                    f"{group_name} 全体試行{global_trial}: 条件ID {row['条件ID']} が条件マスターにありません"
                )
                continue
            if paper_set not in experimenter_orders:
                differences.append(f"{group_name} 全体試行{global_trial}: 未知の用紙名 {paper_set}")
                continue
            if not 1 <= paper_trial <= len(experimenter_orders[paper_set]):
                differences.append(
                    f"{group_name} 全体試行{global_trial}: 用紙内番号が範囲外です: {paper_trial}"
                )
                continue

            experimenter = experimenter_orders[paper_set][paper_trial - 1]
            plan_overtaker = _optional(row["追越者"])
            plan_overtaken = _optional(row["被追越者"])
            plan_solo = _optional(row["単独対象"])
            for source_header, exp_header, plan_value in (
                ("条件ID", "条件ID", row["条件ID"]),
                ("区分", "区分", row["区分"]),
                ("通路幅", "通路幅", row["通路幅"]),
                ("回避", "回避", row["回避"]),
                ("前方速度", "速度", row["前方速度"]),
                ("追越側", "追越方向", row["追越側"]),
                ("追越者", "追越者", row["追越者"]),
                ("被追越者", "被追越者", row["被追越者"]),
                ("単独対象", "単独対象", row["単独対象"]),
            ):
                _require_exact(
                    differences,
                    f"{group_name} 全体試行{global_trial} / {paper_set}_実験者 {paper_trial}",
                    f"グループ表 {source_header}",
                    _optional(plan_value),
                    f"用紙別実験者表 {exp_header}",
                    _optional(experimenter[exp_header]),
                )

            entry = _normalize_plan_fields(
                row,
                group_name=group_name,
                global_trial=global_trial,
                set_number=set_number,
                paper_set=paper_set,
                paper_trial=paper_trial,
                condition=condition,
                print_sheets=print_sheets,
                differences=differences,
            )
            group_entries.append(entry)

            reference = common_record(
                {
                    "グループ": group_name,
                    "全体順": str(global_trial),
                    "第何セット": str(set_number),
                    "用紙": paper_set,
                    "条件ID": row["条件ID"],
                    "区分": row["区分"],
                    "通路幅": row["通路幅"],
                    "回避": row["回避"],
                    "前方速度": row["前方速度"],
                    "追越側": row["追越側"],
                    "追越者": row["追越者"],
                    "被追越者": row["被追越者"],
                    "単独対象": row["単独対象"],
                }
            )
            index = (group_name, global_trial)
            audit_source = additional_ledger if is_additional else base_master
            source_label = "06_追加324試行" if is_additional else "03_全972試行"
            if index not in audit_source:
                differences.append(
                    f"{source_label}: {group_name} 全体試行{global_trial} がありません"
                )
            elif audit_source[index] != reference:
                differences.append(
                    f"{group_name} 全体試行{global_trial}: グループ別表と{source_label}が不一致。"
                    f"グループ別={reference}; 台帳={audit_source[index]}"
                )

        _require_exact(
            differences,
            f"{group_name}",
            "各セット内の用紙内番号",
            [entry["paper_trial_number"] for entry in group_entries],
            "各セット1〜18",
            [trial for _set in range(4) for trial in range(1, 19)],
        )
        group_templates[group_name] = group_entries
        full_plan_records.extend(group_entries)

        basic_entries = [entry for entry in group_entries if entry["trial_type"] == "basic"]
        width_changes = sum(
            before["corridor_width"] != after["corridor_width"]
            for before, after in zip(basic_entries, basic_entries[1:])
        )
        _require_exact(
            differences,
            f"01_グループ順 {group_name}",
            "グループ表から集計した幅変更回数",
            width_changes,
            "Excelの幅変更欄",
            schedule["expected_width_changes"],
        )
        basic_overtakers = {
            subject: sum(entry["overtaker"] == subject for entry in basic_entries)
            for subject in ("被験者1", "被験者2")
        }
        _require_exact(
            differences,
            f"01_グループ順 {group_name}",
            "基本追越者数（被験者1、被験者2）",
            (basic_overtakers["被験者1"], basic_overtakers["被験者2"]),
            "Excelの追越者1・2欄",
            (schedule["expected_basic_overtaker_1"], schedule["expected_basic_overtaker_2"]),
        )
        _require_exact(
            differences,
            f"01_グループ順 {group_name}",
            "基本判定",
            schedule["basic_status"],
            "期待値",
            "OK",
        )
        extra_entries = [entry for entry in group_entries if entry["trial_type"] == "additional"]
        total_overtakers = {
            subject: sum(entry["overtaker"] == subject for entry in basic_entries + extra_entries)
            for subject in ("被験者1", "被験者2")
        }
        _require_exact(
            differences,
            f"01_グループ順 {group_name}",
            "追加後試行数",
            len(basic_entries) + len(extra_entries),
            "Excelの追加後計画試行数",
            schedule["expected_total_after_additional"],
        )
        _require_exact(
            differences,
            f"01_グループ順 {group_name}",
            "追加後の追越者数（被験者1、被験者2）",
            (total_overtakers["被験者1"], total_overtakers["被験者2"]),
            "Excelの追越者1・2計画欄",
            (schedule["expected_total_overtaker_1"], schedule["expected_total_overtaker_2"]),
        )
        _require_exact(
            differences,
            f"01_グループ順 {group_name}",
            "追加後判定",
            schedule["additional_status"],
            "期待値",
            "OK",
        )

    # 追加セット台帳の全行がグループ別シートに存在するか、また追加検算の集計と一致するか。
    group_entry_index = {
        (entry["group"], entry["global_trial_number"]): entry for entry in full_plan_records
    }
    for (group_name, trial_number), ledger_record in additional_ledger.items():
        entry = group_entry_index.get((group_name, trial_number))
        if entry is None:
            differences.append(
                f"06_追加324試行: {group_name} 全体試行{trial_number} がグループ表にありません"
            )
            continue
        plan_record = {
            "group": entry["group"],
            "global_trial_number": entry["global_trial_number"],
            "set_number": entry["set_number"],
            "paper_name": entry["paper_name"],
            "condition_id": entry["condition_id"],
            "condition_category": entry["condition_category"],
            "corridor_width": entry["corridor_width"],
            "avoidance": entry["avoidance"],
            "leading_walker_speed": entry["leading_walker_speed"],
            "overtaking_direction": entry["overtaking_direction"],
            "overtaker": entry["overtaker"],
            "overtaken_person": entry["overtaken_person"],
            "solo_target": entry["solo_target"],
        }
        if plan_record != ledger_record:
            differences.append(
                f"{group_name} 追加試行{trial_number}: 追加台帳とグループ別表が不一致。"
                f"グループ別={plan_record}; 台帳={ledger_record}"
            )

    extra_checks = _table(
        plan_book,
        "07_追加検算",
        (
            "グループ",
            "追加試行",
            "追加追越1",
            "追加追越2",
            "追加被追越1",
            "追加被追越2",
            "追加単独",
            "判定",
        ),
        workbook_name=PLAN_WORKBOOK_NAME,
    )
    extra_checks = {
        row["グループ"]: row for row in extra_checks if re.fullmatch(r"G\d{2}", row["グループ"])
    }
    _require_exact(
        differences, "07_追加検算", "グループ数", len(extra_checks), "期待値", GROUP_COUNT
    )
    for group_name, entries in group_templates.items():
        extra = [entry for entry in entries if entry["trial_type"] == "additional"]
        stats = {
            "追加試行": len(extra),
            "追加追越1": sum(entry["overtaker"] == "被験者1" for entry in extra),
            "追加追越2": sum(entry["overtaker"] == "被験者2" for entry in extra),
            "追加被追越1": sum(entry["overtaken_person"] == "被験者1" for entry in extra),
            "追加被追越2": sum(entry["overtaken_person"] == "被験者2" for entry in extra),
            "追加単独": sum(entry["condition_category"] == "単独歩行" for entry in extra),
        }
        source = extra_checks.get(group_name)
        if source is None:
            differences.append(f"07_追加検算: {group_name} の行がありません")
            continue
        for field, actual in stats.items():
            _require_exact(
                differences,
                f"07_追加検算 {group_name}",
                f"計画から集計した{field}",
                actual,
                "Excel検算値",
                _integer(source[field], f"07_追加検算 {group_name} {field}"),
            )
        _require_exact(
            differences, f"07_追加検算 {group_name}", "判定", source["判定"], "期待値", "OK"
        )

    # 全グループで同一の配布用紙セットの条件順・役割・教示文であることを確認し、
    # 共有テンプレートとしてJSONと監査表に一度だけ格納する。
    set_templates: dict[str, list[dict[str, Any]]] = {}
    for paper_set in PAPER_SETS:
        templates = [
            entry for entry in group_templates.get("G01", []) if entry["paper_name"] == paper_set
        ]
        templates.sort(key=lambda entry: entry["paper_trial_number"])
        if len(templates) != 18:
            differences.append(f"G01 / {paper_set}: 18試行のテンプレートがありません")
        set_templates[paper_set] = templates
        for group_name, entries in group_templates.items():
            rows = sorted(
                (entry for entry in entries if entry["paper_name"] == paper_set),
                key=lambda entry: entry["paper_trial_number"],
            )
            left = [
                (entry["condition_id"], entry["subject_1"], entry["subject_2"]) for entry in rows
            ]
            right = [
                (entry["condition_id"], entry["subject_1"], entry["subject_2"])
                for entry in templates
            ]
            _require_exact(
                differences,
                f"{group_name} / {paper_set}",
                "グループ別配布順",
                left,
                "共通プリントテンプレート",
                right,
            )

    # 失敗した生成で古いJSONを残したまま利用することを防ぐため、矛盾時は書き出さない。
    if differences:
        raise PlanError(
            "Excel間に不一致があります。run_order.jsonは更新していません。\n"
            + "\n".join(f"- {difference}" for difference in differences)
        )

    source_hashes_after = {
        "completed_plan": sha256_file(plan_workbook_path),
        "printable_booklet": sha256_file(print_workbook_path),
    }
    if source_hashes_before != source_hashes_after:
        raise PlanError("照合中に元Excelが変更されました。Excelを閉じてから再実行してください。")
    source_workbooks = {
        "completed_plan": {
            "filename": plan_workbook_path.name,
            "sha256": source_hashes_after["completed_plan"],
        },
        "printable_booklet": {
            "filename": print_workbook_path.name,
            "sha256": source_hashes_after["printable_booklet"],
        },
    }
    groups = {
        group_name: {
            **group_schedules[group_name],
            "basic_trial_count": BASIC_TRIAL_COUNT,
            "additional_trial_count": ADDITIONAL_TRIAL_COUNT,
            "trials": group_templates[group_name],
        }
        for group_name in sorted(group_templates)
    }
    plan: dict[str, Any] = {
        "schema_version": 1,
        "plan_version": PLAN_VERSION,
        "source_workbooks": source_workbooks,
        "group_count": GROUP_COUNT,
        "basic_trial_count": BASIC_TRIAL_COUNT,
        "additional_trial_count": ADDITIONAL_TRIAL_COUNT,
        "groups": groups,
        "set_templates": set_templates,
        "audit": {
            "completed_plan_rows_checked": len(base_master),
            "additional_rows_checked": len(additional_ledger),
            "print_sheets_checked": [
                f"{paper_set}_被験者{subject_number}"
                for paper_set in PAPER_SETS
                for subject_number in (1, 2)
            ],
            "print_rows_checked": len(PAPER_SETS) * 18 * 2,
            "result": "一致",
        },
    }
    plan["plan_sha256"] = canonical_plan_hash(plan)
    report = _build_audit_report(plan, [groups[name] for name in sorted(groups)])
    _write_atomic(Path(plan_path), json.dumps(plan, ensure_ascii=False, indent=2) + "\n")
    _write_atomic(Path(audit_path), report)
    return plan


def _write_atomic(path: Path, content: str) -> None:
    """一時ファイルへ書いてから置換し、途中で壊れた計画ファイルを残さない。"""

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(content, encoding="utf-8")
    os.replace(temporary, path)


def load_run_order(
    plan_path: Path = DEFAULT_PLAN_PATH,
    *,
    input_dir: Path = DEFAULT_INPUT_DIR,
) -> dict[str, Any]:
    """計画JSONの内部ハッシュとExcel原本ハッシュを検証して読み込む。"""

    plan_path = Path(plan_path).resolve()
    try:
        plan = json.loads(plan_path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise PlanError(
            f"順序データがありません: {plan_path}。先にscripts/build_run_order.pyを実行してください。"
        ) from exc
    except (OSError, json.JSONDecodeError) as exc:
        raise PlanError(f"順序データを読めません: {plan_path}: {exc}") from exc
    if not isinstance(plan, dict) or plan.get("plan_sha256") != canonical_plan_hash(plan):
        raise PlanError("run_order.jsonの計画ハッシュが一致しません。再生成してください。")
    for source in plan.get("source_workbooks", {}).values():
        source_path = Path(input_dir) / source["filename"]
        if not source_path.is_file():
            raise PlanError(f"計画元Excelがありません: {source_path}。配置して再生成してください。")
        if sha256_file(source_path) != source["sha256"]:
            raise PlanError(
                f"計画元Excelが生成後に変更されています: {source_path}。"
                "既存進捗へ混ぜないため、内容を照合してから順序データを再生成してください。"
            )
    return plan


def main() -> int:
    try:
        plan = build_run_order()
    except PlanError as exc:
        print(str(exc), file=os.sys.stderr)
        return 1
    print(
        f"run_order.jsonを生成しました: {DEFAULT_PLAN_PATH}\n"
        f"計画SHA-256: {plan['plan_sha256']}\n"
        f"Excel照合表: {DEFAULT_AUDIT_PATH}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
