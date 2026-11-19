#!/usr/bin/env python3
"""Finish YAM pose-based link recovery on the PC after follower motion-limit changes.

Run from yam_ros2_ws Git repository root:
    python yam_follower/scripts/enable_yam_auto_rejoin_pc_FIXED.py

Works with auto_reconnect declared true or false in prior patches.
Does not alter follower motion shaping or hardware fault protections.
"""
from pathlib import Path
import re

ROOT = Path.cwd()
BRIDGE = Path('yam_follower/src/yam_teleop_cpp/src/yam_live_bridge.cpp')
LAUNCH = Path('yam_follower/src/yam_bringup/launch/yam_follower_nuc.launch.py')
WEB = Path('yam_sim_display/dashboard/web/app.js')


def replace_once(src, old, new, what):
    if new in src:
        return src
    if src.count(old) != 1:
        raise RuntimeError(f'{what}: expected exactly one source block, found {src.count(old)}. No files changed.')
    return src.replace(old, new, 1)


def patch_bridge(s):
    # Fail safely when code isn't the existing 5-degree recovery state machine.
    for anchor in (
        'REJOIN_TOLERANCE', 'REJOIN_DWELL',
        'void maybe_rejoin(const Clock::time_point now)',
        'reconnect_pending_ = auto_reconnect_',
        'reconnect_candidate_valid_',
        'if (tracking_) disarm("Follower feedback lost")',
    ):
        if anchor not in s:
            raise RuntimeError(f'Bridge lacks 5-degree recovery state machine ({anchor}). No files changed.')

    # Original scripts supplied opposite defaults. Both are supported.
    old = r'(auto_reconnect_\s*=\s*declare_parameter<bool>\(\s*"auto_reconnect"\s*,\s*)(true|false)(\s*\);)'
    s2, matches = re.subn(old, r'\g<1>true\g<3>', s)
    if matches != 1:
        raise RuntimeError('Cannot locate unique auto_reconnect declaration. No files changed.')
    s = s2

    # One previous version also initialized the field to false; make default
    # true for consistent behavior even before ROS parameters are read.
    s = re.sub(r'(\bbool\s+auto_reconnect_\s*=\s*)false\s*;', r'\g<1>true;', s)

    if '"Bridge ready; auto_track=%s, auto_reconnect=%s, "' not in s:
        s = replace_once(
            s,
            '''      "Bridge ready; auto_track=%s, actuation_authorized=%s; tracking initially OFF",
      auto_track_ ? "true" : "false", actuation_authorized_ ? "true" : "false");''',
            '''      "Bridge ready; auto_track=%s, auto_reconnect=%s, "
      "actuation_authorized=%s; tracking initially OFF",
      auto_track_ ? "true" : "false",
      auto_reconnect_ ? "true" : "false",
      actuation_authorized_ ? "true" : "false");''',
            'startup status logging',
        )

    if '"WAITING_FOR_LINK"' not in s:
        s = replace_once(
            s,
            '''      << " reconnect=" << (reconnect_pending_
           ? (reconnect_candidate_valid_ ? "STABILIZING" : "WAITING_FOR_ALIGNMENT")
           : "IDLE")''',
            '''      << " reconnect=" << (reconnect_pending_
           ? (!fresh(leader_) ? "WAITING_FOR_LINK"
              : (reconnect_candidate_valid_ ? "STABILIZING" : "WAITING_FOR_ALIGNMENT"))
           : "IDLE")''',
            'link recovery status',
        )
    return s


def patch_launch(s):
    if "auto_reconnect = LaunchConfiguration('auto_reconnect').perform(context)" not in s:
        s = replace_once(s,
            "    auto_track = LaunchConfiguration('auto_track').perform(context)\n",
            "    auto_track = LaunchConfiguration('auto_track').perform(context)\n"
            "    auto_reconnect = LaunchConfiguration('auto_reconnect').perform(context)\n",
            'launch parameter read')

    if "'auto_reconnect': auto_reconnect.strip().lower() in ('true','1')" not in s:
        marker = "'auto_track': auto_track.strip().lower() in ('true','1')"
        if s.count(marker) != 1:
            raise RuntimeError('Cannot locate unique auto_track bridge parameter. No files changed.')
        # The marker may be followed by a comma + another parameter in the
        # original launch, OR by }]) after the speed-removal patch.
        start = s.index(marker)
        after = start + len(marker)
        remaining = s[after:]
        if remaining.startswith(','):
            # Insert before any following parameter.
            extra = ",\n                          'auto_reconnect': auto_reconnect.strip().lower() in ('true','1')"
        else:
            extra = ",\n                          'auto_reconnect': auto_reconnect.strip().lower() in ('true','1')"
        s = s[:after] + extra + s[after:]
        # Normalize a doubled comma before another parameter when inserting
        # into the original (pre-motion-removal) launch.
        s = s.replace("'auto_reconnect': auto_reconnect.strip().lower() in ('true','1'),\n,",
                      "'auto_reconnect': auto_reconnect.strip().lower() in ('true','1'),\n")
        # If we inserted before an existing comma on the original line,
        # move that comma to the end of the inserted argument.
        problem = "'auto_reconnect': auto_reconnect.strip().lower() in ('true','1'),\n                          ,"
        s = s.replace(problem, "'auto_reconnect': auto_reconnect.strip().lower() in ('true','1'),\n                          ")

    if "DeclareLaunchArgument('auto_reconnect'" not in s:
        s = replace_once(s,
            "        DeclareLaunchArgument('auto_track', default_value='false',\n"
            "                              description='Auto-arm bridge once leader and follower align'),\n",
            "        DeclareLaunchArgument('auto_track', default_value='false',\n"
            "                              description='Auto-arm bridge once leader and follower align'),\n"
            "        DeclareLaunchArgument('auto_reconnect', default_value='true',\n"
            "                              description='Rejoin on leader-link recovery with all six joints within five degrees'),\n",
            'launch argument')
    else:
        s = re.sub(r"(DeclareLaunchArgument\('auto_reconnect',\s*default_value=')false(')",
                   r'\g<1>true\g<2>', s)
    return s


def patch_web(s):
    if 'bridge.reconnect === "WAITING_FOR_LINK"' in s:
        return s
    pat = r'(?m)^(?P<indent>[ \t]*)const rejoin = bridge\.reconnect === "WAITING_FOR_ALIGNMENT"'
    def add_link_status(m):
        indent = m.group('indent')
        return (f'{indent}const rejoin = bridge.reconnect === "WAITING_FOR_LINK"\n'
                f'{indent}  ? " · Rejoin: waiting for leader connection"\n'
                f'{indent}  : bridge.reconnect === "WAITING_FOR_ALIGNMENT"')
    result, n = re.subn(pat, add_link_status, s)
    if n != 1:
        raise RuntimeError(f'Dashboard 5-degree status block not found (matches={n}). No files changed.')
    return result


def main():
    patches = [(BRIDGE, patch_bridge), (LAUNCH, patch_launch), (WEB, patch_web)]
    prepared = []
    for relative, patch in patches:
        path = ROOT / relative
        if not path.is_file():
            raise SystemExit(f'Cannot find {relative}; run from yam_ros2_ws root.')
        with path.open('r', encoding='utf-8', newline='') as f:
            original = f.read()
        eol = '\r\n' if '\r\n' in original else '\n'
        normalized = original.replace('\r\n', '\n')
        updated = patch(normalized).replace('\n', eol)
        prepared.append((path, original, updated))
    for path, old, new in prepared:
        if old == new:
            print('Already updated:', path.relative_to(ROOT))
        else:
            with path.open('w', encoding='utf-8', newline='') as f:
                f.write(new)
            print('Updated:', path.relative_to(ROOT))
    print('Automatic network rejoin enabled: all six joints within 5 degrees and stable 750 ms.')
    print('Follower motor control, acceleration, speed, and hardware interlocks unchanged.')


if __name__ == '__main__':
    main()
