#!/usr/bin/env python3
"""計画ハッシュごとに試行Takeと録画進捗を保存する。"""

from __future__ import annotations

import json
import os
from datetime import datetime
from pathlib import Path
from typing import Any


class ProgressError(RuntimeError):
    """進捗ファイルを安全に読み書きできない。"""


class RecordingProgress:
    """計画とグループごとにTake履歴を保持し、再録画・未完了試行を選ぶ。"""

    def __init__(self, path: Path, group: str, plan_sha256: str) -> None:
        self.path = Path(path)
        self.group = group
        self.plan_sha256 = plan_sha256
        self.data: dict[str, Any] = {
            "schema_version": 1,
            "group": group,
            "plan_sha256": plan_sha256,
            "additional_enabled": False,
            "additional_decision": "pending",
            "attempts": {},
        }
        if self.path.exists():
            try:
                existing = json.loads(self.path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                raise ProgressError(f"進捗ファイルを読めません: {self.path}: {exc}") from exc
            if existing.get("group") != group or existing.get("plan_sha256") != plan_sha256:
                raise ProgressError(
                    f"進捗ファイルのグループまたは計画ハッシュが一致しません: {self.path}"
                )
            if not isinstance(existing.get("attempts"), dict):
                raise ProgressError(f"進捗ファイルのattemptsが壊れています: {self.path}")
            self.data = existing

    @classmethod
    def open(
        cls,
        output_root: Path,
        group: str,
        plan_sha256: str,
    ) -> "RecordingProgress":
        progress_dir = Path(output_root) / ".experiment_progress"
        filename = f"{group.lower()}_plan_{plan_sha256[:16]}.json"
        return cls(progress_dir / filename, group, plan_sha256)

    @property
    def additional_enabled(self) -> bool:
        return bool(self.data.get("additional_enabled", False))

    def enable_additional(self) -> None:
        self.data["additional_enabled"] = True
        self.data["additional_decision"] = "accepted"
        self.save()

    @property
    def additional_decision(self) -> str:
        return str(self.data.get("additional_decision", "pending"))

    def decline_additional(self) -> None:
        self.data["additional_decision"] = "declined"
        self.save()

    def attempts_for(self, trial_number: int) -> list[dict[str, Any]]:
        attempts = self.data["attempts"].get(str(trial_number), [])
        return attempts if isinstance(attempts, list) else []

    def next_take_number(self, trial_number: int) -> int:
        return len(self.attempts_for(trial_number)) + 1

    def begin_attempt(self, trial_number: int, recording_id: str) -> int:
        take_number = self.next_take_number(trial_number)
        attempts = self.attempts_for(trial_number)
        attempts.append(
            {
                "take_number": take_number,
                "status": "in_progress",
                "recording_id": recording_id,
                "started_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            }
        )
        self.data["attempts"][str(trial_number)] = attempts
        self.save()
        return take_number

    def finish_attempt(
        self,
        trial_number: int,
        take_number: int,
        *,
        success: bool,
        details: str = "",
    ) -> None:
        attempts = self.attempts_for(trial_number)
        for attempt in reversed(attempts):
            if attempt.get("take_number") == take_number:
                attempt["status"] = "complete" if success else "needs_rerecord"
                attempt["finished_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
                if details:
                    attempt["details"] = details
                else:
                    attempt.pop("details", None)
                break
        else:
            raise ProgressError(f"試行{trial_number} Take{take_number}の開始記録がありません")
        self.data["attempts"][str(trial_number)] = attempts
        self.save()

    def completed_trials(self) -> set[int]:
        completed: set[int] = set()
        for trial_text, attempts in self.data["attempts"].items():
            try:
                trial_number = int(trial_text)
            except ValueError:
                continue
            if attempts and attempts[-1].get("status") == "complete":
                completed.add(trial_number)
        return completed

    def rerecord_trials(self) -> list[int]:
        rerecord: list[int] = []
        for trial_text, attempts in self.data["attempts"].items():
            if not attempts:
                continue
            try:
                trial_number = int(trial_text)
            except ValueError:
                continue
            if attempts[-1].get("status") != "complete":
                rerecord.append(trial_number)
        return sorted(rerecord)

    def recommended_trial(
        self,
        available_trials: list[int],
        *,
        after_trial: int | None = None,
    ) -> int | None:
        available = set(available_trials)
        ordered_rerecords = [
            trial for trial in self.rerecord_trials() if trial in available and trial != after_trial
        ]
        if after_trial is not None:
            ordered_rerecords.sort(key=lambda trial: (trial <= after_trial, trial))
        if ordered_rerecords:
            return ordered_rerecords[0]
        completed = self.completed_trials()
        pending = [
            trial for trial in available_trials if trial not in completed and trial != after_trial
        ]
        if after_trial is not None:
            pending.sort(key=lambda trial: (trial <= after_trial, trial))
        return pending[0] if pending else None

    def completed_count(self, available_trials: list[int]) -> int:
        return len(self.completed_trials().intersection(available_trials))

    def save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_name(f".{self.path.name}.tmp")
        try:
            temporary.write_text(
                json.dumps(self.data, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            os.replace(temporary, self.path)
        except OSError as exc:
            raise ProgressError(f"進捗を保存できません: {self.path}: {exc}") from exc
