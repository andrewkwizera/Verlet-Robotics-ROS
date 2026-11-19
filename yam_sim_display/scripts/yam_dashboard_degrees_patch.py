#!/usr/bin/env python3
"""Convert only YAM dashboard presentation values to degrees.

ROS JointState values, dynamics, thresholds, and geometric calculations stay in radians.
No motor control or ROS launch files are changed.
"""
from pathlib import Path
import sys

root = Path.home() / 'yam_ros2_ws' / 'yam_sim_display' / 'dashboard' / 'web'
if len(sys.argv) > 1:
    root = Path(sys.argv[1]).expanduser()
js_file = root / 'app.js'
html_file = root / 'index.html'
if not js_file.is_file() or not html_file.is_file():
    raise SystemExit(f'Cannot find app.js and index.html under {root}')
js = js_file.read_text(encoding='utf-8')
html = html_file.read_text(encoding='utf-8')
if 'function fmtDegrees(' in js:
    print('Dashboard is already patched for degrees; no files changed.')
    raise SystemExit(0)

def once(data, old, new, tag):
    count = data.count(old)
    if count != 1:
        raise RuntimeError(f'Expected one {tag} occurrence; found {count}. No files changed.')
    return data.replace(old, new, 1)

js = once(js,
'''function fmtAge(ms) {''',
'''// Presentation-only conversion. Keep ROS messages, kinematics and safety limits in radians.
function fmtDegrees(radians, digits = 2) {
  if (radians === null || radians === undefined ||
      !Number.isFinite(Number(radians))) return "\\u2014";
  return fmt(Number(radians) * 180 / Math.PI, digits);
}

function fmtAge(ms) {''',
'fmtDegrees helper')
js = once(js,
'''`${fmt(tracking.max_abs_error_rad * 1000, 1)} mrad \\u00b7 RMS `
        + `${fmt(tracking.rms_error_rad * 1000, 1)} mrad`;''',
'''`${fmtDegrees(tracking.max_abs_error_rad)}\\u00b0 \\u00b7 RMS `
        + `${fmtDegrees(tracking.rms_error_rad)}\\u00b0`;''',
'robot summary error')
js = once(js, 'leaderCell.textContent = fmt(leader[name]);',
          'leaderCell.textContent = fmtDegrees(leader[name], 1);', 'leader positions')
js = once(js, 'followerCell.textContent = fmt(follower[name]);',
          'followerCell.textContent = fmtDegrees(follower[name], 1);', 'follower positions')
js = once(js, 'errorCell.textContent = fmt(row.error_rad);',
          'errorCell.textContent = fmtDegrees(row.error_rad);', 'signed tracking error')
js = once(js, 'mradCell.textContent = fmt(row.error_mrad, 1);',
          'mradCell.textContent = fmtDegrees(Math.abs(row.error_rad));', 'absolute tracking error')
js = once(js,
'''? `max ${fmt(tracking.max_abs_error_rad * 1000, 1)} mrad \\u00b7 RMS ${fmt(tracking.rms_error_rad * 1000, 1)} mrad`''',
'''? `max ${fmtDegrees(tracking.max_abs_error_rad)}\\u00b0 \\u00b7 RMS ${fmtDegrees(tracking.rms_error_rad)}\\u00b0`''',
'joint summary error')
js = once(js, '? `${fmt(bridge.max_separation_rad, 3)} rad`',
          '? `${fmtDegrees(bridge.max_separation_rad)}\\u00b0`', 'bridge separation')
for old, new in [
    ('<th>Leader (rad)</th>', '<th>Leader (°)</th>'),
    ('<th>Follower (rad)</th>', '<th>Follower (°)</th>'),
    ('<th>Error (rad)</th>', '<th>Error (°)</th>'),
    ('<th>Error (mrad)</th>', '<th>|Error| (°)</th>'),
]:
    html = once(html, old, new, old)
# Backups only after all expected matches have been verified.
for path in (js_file, html_file):
    backup = path.with_name(path.name + '.before-degrees.bak')
    if not backup.exists():
        backup.write_bytes(path.read_bytes())
js_file.write_text(js, encoding='utf-8')
html_file.write_text(html, encoding='utf-8')
print(f'Updated: {js_file}\nUpdated: {html_file}')
print('ROS control units and safety thresholds are unchanged (radians).')
