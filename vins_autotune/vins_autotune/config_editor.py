"""Read and modify VINS YAML config files for auto-tuning.

Preserves comments and formatting by doing targeted line replacements
rather than full YAML round-trip.
"""
from __future__ import annotations

import os
import re
import shutil
from dataclasses import dataclass
from typing import Any, Dict, List, Optional


@dataclass
class ParamSpec:
    key: str
    value: Any
    min: Any
    max: Any
    step: Any
    description: str = ""
    dtype: str = "float"  # "float", "int", "bool"

    def clamp(self, value: Any) -> Any:
        if self.dtype == "float":
            return float(max(self.min, min(self.max, value)))
        if self.dtype == "int":
            return int(max(self.min, min(self.max, round(value))))
        if self.dtype == "bool":
            return 1 if value else 0
        return value


class ConfigEditor:
    """Edit a VINS YAML config by replacing scalar values in-place.

    Keeps a backup of the original config on first modification.
    """

    def __init__(self, config_path: str):
        self.config_path = os.path.expanduser(config_path)
        self._backup_path = self.config_path + ".autotune_backup"
        self._backed_up = False

    def backup(self) -> None:
        if not self._backed_up and os.path.exists(self.config_path):
            shutil.copy2(self.config_path, self._backup_path)
            self._backed_up = True

    def restore(self) -> None:
        if os.path.exists(self._backup_path):
            shutil.copy2(self._backup_path, self.config_path)

    def read_value(self, key: str) -> Optional[Any]:
        """Read a scalar value for `key` from the YAML file."""
        pattern = re.compile(rf"^\s*{re.escape(key)}\s*:\s*(.+?)(?:\s+#.*)?$")
        with open(self.config_path, "r") as f:
            for line in f:
                m = pattern.match(line)
                if m:
                    raw = m.group(1).strip()
                    try:
                        if "." in raw or "e" in raw.lower():
                            return float(raw)
                        return int(raw)
                    except ValueError:
                        return raw
        return None

    def set_value(self, key: str, value: Any) -> bool:
        """Replace the scalar value of `key` in-place, preserving comments."""
        self.backup()
        pattern = re.compile(rf"^(\s*{re.escape(key)}\s*:\s*)(.+?)(\s+#.*)?$")
        new_lines: List[str] = []
        replaced = False
        with open(self.config_path, "r") as f:
            for line in f:
                m = pattern.match(line)
                if m and not replaced:
                    prefix, _, comment = m.group(1), m.group(2), m.group(3) or ""
                    new_val = self._format_value(value)
                    new_lines.append(f"{prefix}{new_val}{comment}\n")
                    replaced = True
                else:
                    new_lines.append(line)
        if replaced:
            with open(self.config_path, "w") as f:
                f.writelines(new_lines)
        return replaced

    @staticmethod
    def _format_value(value: Any) -> str:
        if isinstance(value, bool):
            return "1" if value else "0"
        if isinstance(value, float):
            if abs(value) < 1.0 and value != 0.0:
                return f"{value:.6f}"
            return f"{value:.4f}"
        if isinstance(value, int):
            return str(value)
        return str(value)

    def set_many(self, updates: Dict[str, Any]) -> Dict[str, bool]:
        """Apply multiple key→value updates. Returns per-key success."""
        results = {}
        for key, val in updates.items():
            results[key] = self.set_value(key, val)
        return results

    def apply_param_specs(self, specs: Dict[str, ParamSpec]) -> Dict[str, bool]:
        """Apply a set of ParamSpec values to the config file."""
        updates = {k: s.clamp(s.value) for k, s in specs.items()}
        return self.set_many(updates)