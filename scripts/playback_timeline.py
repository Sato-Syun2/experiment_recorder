#!/usr/bin/env python3
"""SVO2とrosbag2の記録時刻を読み、再生開始時刻をそろえる。"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


# 2つの記録媒体の時刻を同一のナノ秒軸で保持する。
@dataclass(frozen=True)
class PlaybackTimeline:
    """SVOとrosbagの開始・終了時刻を保持し、再生時の遅延を計算する。"""

    svo_start_ns: int
    svo_end_ns: int
    bag_start_ns: int
    bag_end_ns: int

    @property
    def svo_duration_seconds(self) -> float:
        return max(0.0, (self.svo_end_ns - self.svo_start_ns) / 1_000_000_000.0)

    @property
    def bag_duration_seconds(self) -> float:
        return max(0.0, (self.bag_end_ns - self.bag_start_ns) / 1_000_000_000.0)

    @property
    def svo_start_delay_seconds(self) -> float:
        """早く始まった媒体の開始時刻を基準に、SVOの待機時間を求める。"""

        return max(0.0, (self.svo_start_ns - min(self.svo_start_ns, self.bag_start_ns)) / 1e9)

    @property
    def bag_start_delay_seconds(self) -> float:
        """早く始まった媒体の開始時刻を基準に、rosbagの待機時間を求める。"""

        return max(0.0, (self.bag_start_ns - min(self.svo_start_ns, self.bag_start_ns)) / 1e9)


# SVOのMCAPチャンクとrosbagのmetadata.yamlから時刻を読み取る。
def inspect_svo_timeline(svo_path: Path) -> Optional[tuple[int, int]]:
    """ZEDを起動せずにMCAPチャンクの時刻範囲を読み取る。

    MCAP以外のSVOや時刻のないファイルではNoneを返し、呼び出し側が
    プロセス起動時刻を基準に同期する。
    """

    mcap_magic = b"\x89MCAP0\r\n"
    try:
        with svo_path.open("rb") as stream:
            if stream.read(len(mcap_magic)) != mcap_magic:
                return None
            first_timestamp: Optional[int] = None
            last_timestamp: Optional[int] = None
            while True:
                record_header = stream.read(9)
                if not record_header:
                    break
                if record_header == mcap_magic:
                    break
                if len(record_header) != 9:
                    return None
                opcode = record_header[0]
                body_length = int.from_bytes(record_header[1:9], "little")
                if opcode == 0x06:  # MCAPのチャンク
                    prefix = stream.read(min(body_length, 24))
                    if len(prefix) != min(body_length, 24):
                        return None
                    if len(prefix) >= 16:
                        chunk_start = int.from_bytes(prefix[0:8], "little")
                        chunk_end = int.from_bytes(prefix[8:16], "little")
                        if chunk_end >= chunk_start and chunk_end > 0:
                            first_timestamp = (
                                chunk_start
                                if first_timestamp is None
                                else min(first_timestamp, chunk_start)
                            )
                            last_timestamp = (
                                chunk_end
                                if last_timestamp is None
                                else max(last_timestamp, chunk_end)
                            )
                    stream.seek(body_length - len(prefix), os.SEEK_CUR)
                else:
                    stream.seek(body_length, os.SEEK_CUR)
                if opcode == 0x02:  # MCAPのフッター
                    break
    except (OSError, ValueError):
        return None

    if first_timestamp is None or last_timestamp is None:
        return None
    return first_timestamp, last_timestamp


def inspect_bag_timeline(bag_path: Path) -> Optional[tuple[int, int]]:
    """metadata.yamlからrosbagの開始時刻と録画時間を読み取る。"""

    try:
        metadata = (bag_path / "metadata.yaml").read_text(encoding="utf-8")
    except OSError:
        return None
    start_match = re.search(
        r"(?m)^\s*starting_time:\s*\n\s*nanoseconds_since_epoch:\s*(\d+)\s*$",
        metadata,
    )
    duration_match = re.search(r"(?m)^\s*duration:\s*\n\s*nanoseconds:\s*(\d+)\s*$", metadata)
    if start_match is None or duration_match is None:
        return None
    start_ns = int(start_match.group(1))
    end_ns = start_ns + int(duration_match.group(1))
    return start_ns, end_ns


def inspect_playback_timeline(svo_path: Path, bag_path: Path) -> Optional[PlaybackTimeline]:
    """両方の記録時刻が読めた場合だけ、共通の時刻軸を返す。"""

    svo_range = inspect_svo_timeline(svo_path)
    bag_range = inspect_bag_timeline(bag_path)
    if svo_range is None or bag_range is None:
        return None
    return PlaybackTimeline(
        svo_start_ns=svo_range[0],
        svo_end_ns=svo_range[1],
        bag_start_ns=bag_range[0],
        bag_end_ns=bag_range[1],
    )
