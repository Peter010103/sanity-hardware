"""Standalone checkout paths. Resolving a configuration performs no device I/O."""
from pathlib import Path
import json
ROOT = next(p for p in Path(__file__).resolve().parents if (p / 'loco_mjx').is_dir())

def resolve_configuration(value):
    if isinstance(value, dict):
        return {key: resolve_configuration(item) for key, item in value.items()}
    if isinstance(value, list):
        return [resolve_configuration(item) for item in value]
    if isinstance(value, str) and value.startswith(('Sanity/', 'loco_mjx/', 'releases/')) and not any(c in value for c in ('\n', ' ')):
        return str(ROOT / value)
    return value

def load_json(path):
    return resolve_configuration(json.loads(Path(path).read_text()))
