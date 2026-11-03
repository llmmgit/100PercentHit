import copy
import os
import re
import stat
import tempfile
import yaml

EDITABLE_SECTIONS = ('vision', 'lateral_tracking', 'imu', 'oac')


def load_config(path):
    """只读取local.yaml，取消runtime.yaml覆盖机制。"""
    path = os.path.abspath(path)
    with open(path, encoding='utf-8') as handle:
        cfg = yaml.safe_load(handle) or {}
    cfg['_config_path'] = path
    return cfg


def runtime_snapshot(cfg):
    """保留现有API名称，返回网页允许编辑的配置分区。"""
    return {name: copy.deepcopy(cfg.get(name, {})) for name in EDITABLE_SECTIONS}


def _yaml_scalar(value):
    text = yaml.safe_dump(value, allow_unicode=True, default_flow_style=True).strip()
    return text[:-3].rstrip() if text.endswith('...') else text


def _update_section(text, section, values):
    lines = text.splitlines(keepends=True)
    section_start = None
    section_end = len(lines)
    for index, line in enumerate(lines):
        if re.match(rf'^{re.escape(section)}\s*:\s*(?:#.*)?$', line.rstrip('\r\n')):
            section_start = index
            break
    if section_start is None:
        if text and not text.endswith('\n'):
            lines[-1] += '\n'
        lines.append(f'{section}:\n')
        section_start = len(lines) - 1
    for index in range(section_start + 1, len(lines)):
        raw = lines[index].rstrip('\r\n')
        if raw and not raw[0].isspace() and re.match(r'^[A-Za-z_][A-Za-z0-9_]*\s*:', raw):
            section_end = index
            break
    found = set()
    for index in range(section_start + 1, section_end):
        match = re.match(r'^(\s+)([A-Za-z_][A-Za-z0-9_]*)(\s*:\s*)(.*?)(\s+#.*)?(\r?\n)?$', lines[index])
        if not match:
            continue
        key = match.group(2)
        if key not in values:
            continue
        comment = match.group(5) or ''
        newline = match.group(6) or '\n'
        lines[index] = f'{match.group(1)}{key}{match.group(3)}{_yaml_scalar(values[key])}{comment}{newline}'
        found.add(key)
    missing = [key for key in values if key not in found]
    if missing:
        lines[section_end:section_end] = [f'  {key}: {_yaml_scalar(values[key])}\n' for key in missing]
    return ''.join(lines)


def save_local(cfg):
    """原子更新local.yaml，仅修改网页可编辑分区，并保留其余注释和排版。"""
    path = cfg['_config_path']
    with open(path, encoding='utf-8') as handle:
        text = handle.read()
    for section, values in runtime_snapshot(cfg).items():
        text = _update_section(text, section, values)
    info = os.stat(path)
    directory = os.path.dirname(path) or '.'
    fd, temporary = tempfile.mkstemp(prefix='local-', suffix='.yaml', dir=directory, text=True)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(temporary, stat.S_IMODE(info.st_mode))
        try:
            os.chown(temporary, info.st_uid, info.st_gid)
        except PermissionError:
            pass
        backup = path + '.bak'
        with open(path, 'rb') as source, open(backup, 'wb') as target:
            target.write(source.read())
        os.chmod(backup, stat.S_IMODE(info.st_mode))
        try:
            os.chown(backup, info.st_uid, info.st_gid)
        except PermissionError:
            pass
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
