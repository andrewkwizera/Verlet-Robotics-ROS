"""Shared result object returned by every dimension evaluator."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

from .reasons import Reason


@dataclass
class DimensionResult:
    state: str
    reasons: List[Reason] = field(default_factory=list)
    evidence: dict = field(default_factory=dict)
    extra: dict = field(default_factory=dict)
