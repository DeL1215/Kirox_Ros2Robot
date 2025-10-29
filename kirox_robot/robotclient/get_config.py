#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations
from dataclasses import dataclass
from datetime import datetime
from typing import Optional
import requests

API_URL = "https://api.xbotworks.com/api/v1/data/get-robot-config"

def _parse_iso_dt(s: Optional[str]) -> Optional[datetime]:
    if not s:
        return None
    try:
        if s.endswith("Z"):
            s = s.replace("Z", "+00:00")
        return datetime.fromisoformat(s)
    except Exception:
        return None

@dataclass
class RobotConfig:
    robotid: str
    userid: str
    voiceid: Optional[int]
    voicename: Optional[str]          # 直接由 API 提供
    promptstyle: Optional[str]
    robotname: Optional[str]
    status: bool
    updatedtime: Optional[datetime]
    message: Optional[str] = None

    # ↓ 新增這兩個，保持 Optional，不會破你現在的伺服器回傳
    language: Optional[str] = None
    frames_per_buffer: Optional[int] = None

    @classmethod
    def from_api(
        cls,
        robotid: str,
        url: str = API_URL,
        timeout: float = 6.0
    ) -> "RobotConfig":
        resp = requests.post(url, json={"robotid": robotid}, timeout=timeout)
        resp.raise_for_status()
        data = resp.json()
        if not isinstance(data, dict):
            raise ValueError(f"Bad response: {data!r}")
        if data.get("robotid") != robotid:
            raise ValueError(f"robotid mismatch: {data.get('robotid')} != {robotid}")

        return cls(
            robotid=data.get("robotid", ""),
            userid=data.get("userid", ""),
            voiceid=data.get("voiceid"),
            voicename=data.get("voicename"),
            promptstyle=data.get("promptstyle"),
            robotname=data.get("robotname"),
            status=bool(data.get("status", True)),
            updatedtime=_parse_iso_dt(data.get("updatedtime")),
            message=data.get("message"),

            # optional: 如果 API 沒給就會是 None
            language=data.get("language"),
            frames_per_buffer=data.get("frames_per_buffer"),
        )

def get_robot_config(
    robotid: str,
    url: str = API_URL,
    timeout: float = 6.0
) -> RobotConfig:
    """便利函式：傳入 robotid，回傳 RobotConfig 物件"""
    return RobotConfig.from_api(robotid, url=url, timeout=timeout)
 