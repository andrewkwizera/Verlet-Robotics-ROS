#!/usr/bin/env python3
"""Install missing 5-degree / 750-ms YAM automatic link-loss recovery.

Run ONLY from the yam_ros2_ws Git root on your Windows PC:
    python yam_follower/scripts/repair_yam_reconnection_pc.py

Handles a bridge with *or without* the prior 5-degree recovery code.
Does not modify motor hardware, follower motion shaping, or follower_speed.
Every file is prepared and validated before any file is written.
"""
from pathlib import Path
import re

ROOT = Path.cwd()
BRIDGE = Path('yam_follower/src/yam_teleop_cpp/src/yam_live_bridge.cpp')
LAUNCH = Path('yam_follower/src/yam_bringup/launch/yam_follower_nuc.launch.py')
STATUS = Path('yam_sim_display/dashboard/bridge_status.py')
WEB = Path('yam_sim_display/dashboard/web/app.js')


def once(text, before, after, name):
    if after in text:
        return text
    n = text.count(before)
    if n != 1:
        raise RuntimeError(f'{name}: expected one insertion point, found {n}; no files modified')
    return text.replace(before, after, 1)


def after_line(text, rx, addition, name):
    matches = list(re.finditer(rx, text, re.M))
    if len(matches) != 1:
        raise RuntimeError(f'{name}: expected one matching line, found {len(matches)}; no files modified')
    pos = matches[0].end()
    return text[:pos] + addition + text[pos:]


REJOIN_METHOD = '''  // Only a LEADER stream timeout during an armed session permits auto-rejoin.
  // Manual disarm, large separation and missing follower feedback never do.
  void maybe_rejoin(const Clock::time_point now) {
    if (!reconnect_pending_ || tracking_) return;
    if (!auto_reconnect_ || !actuation_authorized_ || !fresh(follower_) ||
        cmd_pub_->get_subscription_count() == 0) {
      reconnect_pending_ = false;
      reconnect_candidate_valid_ = false;
      return;
    }
    if (!fresh(leader_)) {
      reconnect_candidate_valid_ = false;
      return;
    }
    for (std::size_t i = 0; i < N; ++i) {
      if (std::abs(leader_.q[i] - follower_.q[i]) > REJOIN_TOLERANCE) {
        reconnect_candidate_valid_ = false;
        return;
      }
    }
    if (!reconnect_candidate_valid_) {
      reconnect_candidate_valid_ = true;
      reconnect_candidate_since_ = now;
      reconnect_leader_pose_ = leader_.q;
      reconnect_follower_pose_ = follower_.q;
      return;
    }
    for (std::size_t i = 0; i < N; ++i) {
      if (std::abs(leader_.q[i] - reconnect_leader_pose_[i]) >
            REJOIN_LEADER_STABILITY ||
          std::abs(follower_.q[i] - reconnect_follower_pose_[i]) >
            REJOIN_FOLLOWER_STABILITY) {
        reconnect_candidate_valid_ = false;
        return;
      }
    }
    if (now - reconnect_candidate_since_ < REJOIN_DWELL) return;
    std::string reason;
    if (arm_tracking(reason)) {
      reconnect_pending_ = false;
      reconnect_candidate_valid_ = false;
      RCLCPP_WARN(get_logger(),
        "AUTO-REJOIN: all 6 joints within 5 deg, stable 750 ms; tracking restored");
    } else {
      reconnect_candidate_valid_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Auto-rejoin blocked: %s", reason.c_str());
    }
  }

'''


def patch_bridge(s):
    # Add/enable parameter, independent of whether the recovery machine exists.
    parameter_re = r'(auto_reconnect_\s*=\s*declare_parameter<bool>\(\s*"auto_reconnect"\s*,\s*)(?:true|false)(\s*\);)'
    if re.search(parameter_re, s):
        s = re.sub(parameter_re, r'\g<1>true\g<2>', s, count=1)
    else:
        s = after_line(s, r'^\s*auto_track_ = declare_parameter<bool>\("auto_track", false\);\n',
            '    auto_reconnect_ = declare_parameter<bool>("auto_reconnect", true);\n',
            'auto_reconnect parameter')

    if 'REJOIN_TOLERANCE' not in s:
        s = after_line(s, r'^\s*static constexpr double SEPARATION_LIMIT = [^\n]+\n',
            '  // Auto-rejoin gate: five degrees per joint, stable for 750 ms.\n'
            '  static constexpr double REJOIN_TOLERANCE = 0.08726646259971647;\n'
            '  static constexpr double REJOIN_LEADER_STABILITY = 0.008726646259971648;\n'
            '  static constexpr double REJOIN_FOLLOWER_STABILITY = 0.017453292519943295;\n'
            '  static constexpr auto REJOIN_DWELL = 750ms;\n',
            'rejoin constants')
    for symbol in ('REJOIN_TOLERANCE', 'REJOIN_LEADER_STABILITY',
                   'REJOIN_FOLLOWER_STABILITY', 'REJOIN_DWELL'):
        if f'{symbol} =' not in s:
            raise RuntimeError(f'Incomplete recovery constants: {symbol}; no files modified')

    if 'void maybe_rejoin(' not in s:
        s = once(s, '  void tick() {\n', REJOIN_METHOD + '  void tick() {\n', 'rejoin method')

    # Make manual disarm cancel pending recovery.
    if 'if (!req->data)' not in s:
        raise RuntimeError('Missing tracking service; no files modified')
    a = s.index('if (!req->data)')
    b = s.index('disarm("Operator requested tracking disabled")', a)
    if 'reconnect_pending_ = false;' not in s[a:b]:
        s = s[:b] + 'reconnect_pending_ = false;\n          reconnect_candidate_valid_ = false;\n          ' + s[b:]

    # Make explicit operator arming clear the recovery state.
    anchor = 'res->message = "TRACKING ARMED: move the physical leader slowly";'
    if anchor not in s:
        raise RuntimeError('Missing tracking service arm result; no files modified')
    arm_section_start = s.index('if (!arm_tracking(reason))')
    arm_section_end = s.index(anchor, arm_section_start)
    if 'reconnect_pending_ = false;' not in s[arm_section_start:arm_section_end]:
        s = s[:arm_section_end] + 'reconnect_pending_ = false;\n        reconnect_candidate_valid_ = false;\n        ' + s[arm_section_end:]

    # Ensure the recovery state is cancelled for missing follower feedback.
    fault_anchor = 'if (tracking_) disarm("Follower feedback lost");'
    if fault_anchor not in s:
        raise RuntimeError('Missing follower feedback watchdog; no files modified')
    a = s.index(fault_anchor)
    b = s.index('return;', a)
    if 'reconnect_pending_ = false;' not in s[a:b]:
        s = s[:a + len(fault_anchor)] + '\n      reconnect_pending_ = false;\n      reconnect_candidate_valid_ = false;' + s[a + len(fault_anchor):]

    # One, and only one, recoverable fault: loss of leader stream while armed.
    timeout_anchor = 'disarm("Leader stream stale");'
    if timeout_anchor not in s:
        raise RuntimeError('Missing leader timeout check; no files modified')
    a = s.index(timeout_anchor)
    b = s.index('auto_armed_once_ = true;', a)
    if 'reconnect_pending_ = auto_reconnect_;' not in s[a:b]:
        s = s[:a + len(timeout_anchor)] + '\n        reconnect_pending_ = auto_reconnect_;\n        reconnect_candidate_valid_ = false;' + s[a + len(timeout_anchor):]

    # Separation faults cannot recover automatically.
    separation_anchor = 'disarm(reason.str());'
    if separation_anchor not in s:
        raise RuntimeError('Missing separation fault disarm; no files modified')
    a = s.index(separation_anchor)
    b = s.index('auto_armed_once_ = true;', a)
    if 'reconnect_pending_ = false;' not in s[a:b]:
        s = s[:a + len(separation_anchor)] + '\n          reconnect_pending_ = false;\n          reconnect_candidate_valid_ = false;' + s[a + len(separation_anchor):]

    # The controller publishes a hold setpoint while unarmed; call rejoin
    # before publishing so command output remains continuous.
    if 'maybe_rejoin(now);' not in s:
        s = once(s, '    std_msgs::msg::Float64MultiArray output;\n',
            '    maybe_rejoin(now);\n    std_msgs::msg::Float64MultiArray output;\n', 'tick rejoin call')

    # Dashboard status state machine.
    if '<< " reconnect="' not in s:
        s = once(s, '      << " leader_age_ms=" << age_ms(leader_)',
            '      << " reconnect=" << (reconnect_pending_\n'
            '           ? (!fresh(leader_) ? "WAITING_FOR_LINK"\n'
            '              : (reconnect_candidate_valid_ ? "STABILIZING" : "WAITING_FOR_ALIGNMENT"))\n'
            '           : "IDLE")\n'
            '      << " leader_age_ms=" << age_ms(leader_)', 'bridge reconnect status')
    else:
        # Existing status format may lack WAITING_FOR_LINK; normalize only
        # the compact status ternary, never touch the command path.
        start = s.index('      << " reconnect="')
        end = s.index('      << " leader_age_ms="', start)
        s = s[:start] + (
            '      << " reconnect=" << (reconnect_pending_\n'
            '           ? (!fresh(leader_) ? "WAITING_FOR_LINK"\n'
            '              : (reconnect_candidate_valid_ ? "STABILIZING" : "WAITING_FOR_ALIGNMENT"))\n'
            '           : "IDLE")\n') + s[end:]

    if 'bool auto_reconnect_' not in s:
        s = once(s, '  bool auto_track_ = false;\n',
            '  bool auto_track_ = false;\n'
            '  bool auto_reconnect_ = true;\n', 'auto-reconnect field')
    else:
        s = re.sub(r'(\bbool\s+auto_reconnect_\s*=\s*)false\s*;', r'\g<1>true;', s)
    if 'bool reconnect_pending_' not in s:
        s = once(s, '  bool tracking_ = false;\n',
            '  bool tracking_ = false;\n'
            '  bool reconnect_pending_ = false;\n'
            '  bool reconnect_candidate_valid_ = false;\n'
            '  Clock::time_point reconnect_candidate_since_{};\n'
            '  std::array<double,N> reconnect_leader_pose_{};\n'
            '  std::array<double,N> reconnect_follower_pose_{};\n',
            'reconnect member state')
    for symbol in ('reconnect_candidate_valid_', 'reconnect_candidate_since_',
                   'reconnect_leader_pose_', 'reconnect_follower_pose_'):
        if s.count(symbol) < 2:
            raise RuntimeError(f'Incomplete reconnect state: {symbol}; no files modified')

    if '"Bridge ready; auto_track=%s, auto_reconnect=%s, "' not in s:
        old = ('      "Bridge ready; auto_track=%s, actuation_authorized=%s; tracking initially OFF",\n'
               '      auto_track_ ? "true" : "false", actuation_authorized_ ? "true" : "false");')
        new = ('      "Bridge ready; auto_track=%s, auto_reconnect=%s, "\n'
               '      "actuation_authorized=%s; tracking initially OFF",\n'
               '      auto_track_ ? "true" : "false",\n'
               '      auto_reconnect_ ? "true" : "false",\n'
               '      actuation_authorized_ ? "true" : "false");')
        if old in s:
            s = s.replace(old, new, 1)

    # The user's previously removed motion-rate constraints must stay removed.
    # We do not add or modify any speed-limit, acceleration or trajectory code.
    return s


def patch_launch(s):
    read = "    auto_reconnect = LaunchConfiguration('auto_reconnect').perform(context)\n"
    if read not in s:
        s = once(s,
            "    auto_track = LaunchConfiguration('auto_track').perform(context)\n",
            "    auto_track = LaunchConfiguration('auto_track').perform(context)\n" + read,
            'launch reconnect argument read')

    key = "'auto_reconnect': auto_reconnect.strip().lower() in ('true','1')"
    if key not in s:
        # Add exactly one parameter after auto_track in both speed-limited and
        # speed-limit-removed launch formats, without creating a doubled comma.
        pat = r"(?P<auto>'auto_track':\s*auto_track\.strip\(\)\.lower\(\)\s+in\s+\('true','1'\))(?P<comma>,?)"
        match = list(re.finditer(pat, s))
        if len(match) != 1:
            raise RuntimeError('Cannot locate unique auto_track launch parameter; no files modified')
        found = match[0]
        inserted = found.group('auto') + ",\n                          " + key + found.group('comma')
        s = s[:found.start()] + inserted + s[found.end():]

    if "DeclareLaunchArgument('auto_reconnect'" not in s:
        s = once(s,
            "        DeclareLaunchArgument('auto_track', default_value='false',\n"
            "                              description='Auto-arm bridge once leader and follower align'),\n",
            "        DeclareLaunchArgument('auto_track', default_value='false',\n"
            "                              description='Auto-arm bridge once leader and follower align'),\n"
            "        DeclareLaunchArgument('auto_reconnect', default_value='true',\n"
            "                              description='Auto-rejoin after leader link loss and all joints align within 5 degrees'),\n",
            'launch reconnect argument declaration')
    else:
        s = re.sub(r"(DeclareLaunchArgument\('auto_reconnect',\s*default_value=')false(')",
                   r'\g<1>true\g<2>', s)
    return s


def patch_status(s):
    if '"reconnect": fields.get("reconnect"' not in s:
        s = once(s, '        "last_disarm_reason": fields.get("last_disarm_reason", "none"),\n',
            '        "last_disarm_reason": fields.get("last_disarm_reason", "none"),\n'
            '        "reconnect": fields.get("reconnect", "IDLE"),\n', 'status parser')
    return s


def patch_web(s):
    if 'bridge.reconnect === "WAITING_FOR_LINK"' not in s:
        snippet = ('''      const rejoin = bridge.reconnect === "WAITING_FOR_LINK"
        ? " · Rejoin: waiting for leader connection"
        : bridge.reconnect === "WAITING_FOR_ALIGNMENT"
          ? " · Rejoin: align all joints within 5°"
          : bridge.reconnect === "STABILIZING"
            ? " · Rejoin: hold still for 0.75 s"
            : "";
''')
        if re.search(r'\bconst rejoin\s*=', s):
            pattern = r'(?m)^[ \t]*const rejoin =(?:(?!\n[ \t]*detail\.textContent =).)*?;\n'
            matches = list(re.finditer(pattern, s, re.S))
            if len(matches) != 1:
                raise RuntimeError('Cannot locate complete dashboard rejoin display; no files modified')
            s = s[:matches[0].start()] + snippet + s[matches[0].end():]
        else:
            s = once(s, '      detail.textContent =\n        `Actuation: ${author}',
                snippet + '      detail.textContent =\n        `Actuation: ${author}', 'dashboard status display')

    if '`Follower: ${bridge.follower} · Separation: ${gap}` + rejoin +' not in s:
        s = once(s,
            '`Follower: ${bridge.follower} · Separation: ${gap}` +\n',
            '`Follower: ${bridge.follower} · Separation: ${gap}` + rejoin +\n',
            'dashboard rejoin concatenation')
    return s


def main():
    patches = [(BRIDGE, patch_bridge), (LAUNCH, patch_launch),
               (STATUS, patch_status), (WEB, patch_web)]
    prepared = []
    for relative, transform in patches:
        path = ROOT / relative
        if not path.is_file():
            raise SystemExit(f'Missing {relative}; run at yam_ros2_ws root.')
        with path.open('r', encoding='utf-8', newline='') as f:
            raw = f.read()
        ending = '\r\n' if '\r\n' in raw else '\n'
        updated = transform(raw.replace('\r\n', '\n')).replace('\n', ending)
        prepared.append((path, raw, updated))
    # No partial edits if a precondition is unmet.
    for path, before, after in prepared:
        if before == after:
            print('Already updated:', path.relative_to(ROOT))
        else:
            with path.open('w', encoding='utf-8', newline='') as f:
                f.write(after)
            print('Updated:', path.relative_to(ROOT))
    print('Recovery installed: leader-network loss; all 6 joints within 5 degrees; 750 ms stable.')
    print('Follower motion-limit removal is not modified.')


if __name__ == '__main__':
    main()
