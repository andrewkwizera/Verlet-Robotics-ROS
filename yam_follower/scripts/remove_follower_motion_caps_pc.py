#!/usr/bin/env python3
"""Remove bridge/hardware software velocity + acceleration shaping in follower.

This is NOT a safety-interlock bypass: retains joint bounds, position-error
fault guards, CAN timeouts, calibration/estop gates, and factory motor limits.
Run from yam_ros2_ws repository root on PC, then review git diff before push.
"""
from pathlib import Path
import re

ROOT = Path.cwd()
FILES = {
    "bridge": Path("yam_follower/src/yam_teleop_cpp/src/yam_live_bridge.cpp"),
    "hardware": Path("yam_follower/src/yam_ros2_control/src/yam_live_hardware.cpp"),
    "header": Path("yam_follower/src/yam_ros2_control/include/yam_ros2_control/yam_live_hardware.hpp"),
    "follower_launch": Path("yam_follower/src/yam_bringup/launch/yam_follower_nuc.launch.py"),
    "legacy_launch": Path("yam_follower/src/yam_bringup/launch/yam_live_teleop.launch.py"),
    "urdf": Path("yam_follower/src/yam_description/urdf/yam_live.urdf.xacro"),
}


def once(text, old, new, label):
    if old not in text:
        if new and new in text:
            return text
        raise RuntimeError(f"Unexpected source at {label}: target block not found")
    if text.count(old) != 1:
        raise RuntimeError(f"Ambiguous block at {label}")
    return text.replace(old, new, 1)


def bridge(s):
    s = once(s,
'''    command_rate_ = declare_parameter<double>("follower_speed", 0.25);
    if (!std::isfinite(command_rate_) || command_rate_ < 0.05 || command_rate_ > 0.50) {
      throw std::runtime_error("follower_speed must be in [0.05, 0.50] rad/s");
    }
''', '', 'bridge speed parameter')
    s = once(s,
'''  // Avoid double slew limiting at the ROS bridge. The independent hardware
  // rate/acceleration limits and 0.20-rad tracking-error guard remain active.
  // A modest feedback-relative lead keeps commands well inside that guard.
  static constexpr double MAX_BRIDGE_COMMAND_LEAD = 0.07; // rad
''', '', 'bridge command lead cap')
    s = once(s,
'''                 << " rad (configured command speed " << command_rate_ << " rad/s)";''',
'''                 << " rad";''', 'bridge disarm message')
    s = once(s,
'''          // Low-lag target projection: eliminate the bridge's extra 0.50-rad/s
          // slew stage, but never send a sudden large position jump to the
          // follower controller. Hardware still enforces speed, acceleration,
          // tracking-error and joint/actuation limits independently.
          for (std::size_t i=0; i<N; ++i) {
            const auto difference = leader_.q[i] - follower_.q[i];
            command_[i] = follower_.q[i] + std::clamp(difference,
              -MAX_BRIDGE_COMMAND_LEAD, MAX_BRIDGE_COMMAND_LEAD);
          }
''',
'''          // Direct position mirroring: no bridge velocity/acceleration ramp.
          // Hardware fault and operating-envelope checks remain active.
          command_ = leader_.q;
''', 'bridge direct command')
    s = once(s, '''      << " follower_speed_rad_s=" << command_rate_\n''', '', 'bridge speed status')
    s = once(s, '''  double command_rate_ = 0.25;\n''', '', 'bridge speed field')
    return s


def hardware(s):
    s = once(s,
'''    follow_speed_limit_ = std::stod(info_.hardware_parameters.at("follower_speed"));
    if (!std::isfinite(follow_speed_limit_) ||
        follow_speed_limit_ < 0.05 || follow_speed_limit_ > 0.50) {
      throw std::runtime_error(
        "follower_speed must be 0.05..0.50 rad/s; this is a software command limit, not a verified safe motor speed");
    }
''', '', 'hardware speed parameter')
    s = re.sub(r'^[ \t]*follower_speed_\.fill\(0\.0\);[ \t]*\n', '', s, flags=re.M)
    s = re.sub(r'^[ \t]*follower_setpoint_ = positions_;[^\n]*\n', '', s, flags=re.M)
    s = once(s,
'''        // TWO different quantities must not be confused:
        //   requested - measured: whole-loop tracking error (fault guard)
        //   requested - setpoint: remaining distance in the commanded trajectory
        // The previous implementation reset the target to measured + 0.005 rad
        // EVERY 20-ms cycle; that capped the motor's proportional correction to
        // ~0.4 Nm (large joints) or ~0.05 Nm (small joints) and the follower
        // could drift instead of moving toward the leader. Here, the target
        // progresses cumulatively at a bounded joint speed.
''',
'''        // Position mirroring without software speed/acceleration trajectory
        // shaping. Retain the existing independent feedback-error fault guard.
''', 'hardware trajectory comment')
    begin = s.index('        // Speed and acceleration are bounded; velocity feedforward is sent')
    end = s.index('      }\n    }\n    // Encode all six', begin)
    old = s[begin:end]
    expected = ('const double remaining', 'FOLLOW_ACCEL_LIMIT',
                'const double servo_error', 'outputs[i] = {')
    if not all(token in old for token in expected):
        raise RuntimeError('Unexpected hardware trajectory block; no files written')
    new = '''        // Request the leader position directly. Zero velocity feedforward
        // does not impose a software speed limit: the MIT position servo
        // acts on position error, subject to motor firmware and fault checks.
        const double absolute_motor_target = requested[i] + offsets_[i];
        outputs[i] = {
          std::remainder(absolute_motor_target, MOTOR_RANGE), 0.0,
          kp_[i], kd_[i], gravity[i]};
'''
    s = s[:begin] + new + s[end:]
    return s


def header(s):
    return once(s,
'''  // Persistent joint-space servo target; never reset it to measured feedback each cycle.
  std::array<double,N> follower_setpoint_{};
''', '', 'follower trajectory field') .replace(
'''  double follow_speed_limit_ = 0.25;  // rad/s; configurable only at startup
  static constexpr double FOLLOW_ACCEL_LIMIT = 2.0;  // rad/s^2, conservative ramp
  std::array<double, N> follower_speed_{ };
''', '')


def follower_launch(s):
    s = once(s,
'''    speed_string = LaunchConfiguration('follower_speed').perform(context)
    speed = float(speed_string)
    if not math.isfinite(speed) or not 0.05 <= speed <= 0.50:
        raise ValueError('follower_speed must be finite in 0.05..0.50 rad/s')

''', '', 'follower launch speed check')
    s = once(s, '''        'follower_speed:=' + speed_string,\n''', '', 'follower URDF speed arg')
    s = once(s,
'''                          'auto_track': auto_track.strip().lower() in ('true','1'),
                          'follower_speed': speed}]),''',
'''                          'auto_track': auto_track.strip().lower() in ('true','1')}]),''', 'bridge launch speed arg')
    s = once(s,
'''        DeclareLaunchArgument('follower_speed', default_value='0.25',
                              description='rad/s, requires load-specific physical commissioning'),
''', '', 'follower speed CLI arg')
    s = once(s, 'import math\n', '', 'obsolete math import')
    return s


def legacy_launch(s):
    s = once(s,
'''    follower_speed = LaunchConfiguration("follower_speed").perform(context)
    # Do not silently permit an unlimited motor speed. Value is also checked
    # independently by each hardware plugin and the ROS tracking bridge.
    speed = float(follower_speed)
    if not (0.05 <= speed <= 0.50):
        raise ValueError("follower_speed must be 0.05..0.50 rad/s")
''', '', 'legacy launch speed')
    s = once(s,
'''                f"emergency_stop_verified:={estop}",
                f"follower_speed:={follower_speed}"]''',
'''                f"emergency_stop_verified:={estop}"]''', 'legacy URDF arg')
    s = once(s,
'''                                   "auto_track": auto_track.lower() in ("true", "1"),
                                   "follower_speed": speed}]))''',
'''                                   "auto_track": auto_track.lower() in ("true", "1")}]))''', 'legacy bridge arg')
    s = once(s,
'''        DeclareLaunchArgument("follower_speed", default_value="0.25",
                              description="Follower joint speed limit rad/s (0.05..0.50); increase ONLY after verifying physical tracking and load"),
''', '', 'legacy CLI speed')
    return s


def urdf(s):
    s = once(s, '  <xacro:arg name="follower_speed" default="0.25"/>\n', '', 'URDF speed arg')
    s = once(s, '      <param name="follower_speed">$(arg follower_speed)</param>\n', '', 'URDF speed param')
    return s


PATCHERS = dict(bridge=bridge, hardware=hardware, header=header,
                follower_launch=follower_launch, legacy_launch=legacy_launch, urdf=urdf)


def main():
    if not all((ROOT / p).is_file() for p in FILES.values()):
        raise SystemExit('Run from yam_ros2_ws root; expected source files not found')
    changes = []
    for key, rel in FILES.items():
        path = ROOT / rel
        with path.open('r', encoding='utf-8', newline='') as f:
            original = f.read()
        eol = '\r\n' if '\r\n' in original else '\n'
        normalized = original.replace('\r\n', '\n')
        if key == 'hardware' and 'FOLLOW_ACCEL_LIMIT' not in normalized:
            updated = normalized  # previously applied
        elif key == 'bridge' and 'MAX_BRIDGE_COMMAND_LEAD' not in normalized:
            updated = normalized
        elif key == 'header' and 'FOLLOW_ACCEL_LIMIT' not in normalized:
            updated = normalized
        elif key in ('follower_launch', 'legacy_launch', 'urdf') and 'follower_speed' not in normalized:
            updated = normalized
        else:
            updated = PATCHERS[key](normalized)
        changes.append((path, original, updated.replace('\n', eol)))
    for path, old, new in changes:
        if old == new:
            print('Already updated:', path.relative_to(ROOT))
            continue
        with path.open('w', encoding='utf-8', newline='') as f:
            f.write(new)
        print('Updated:', path.relative_to(ROOT))
    print('Software follower speed/acceleration shaping removed.')
    print('Safety stops and motor firmware limits retained.')


if __name__ == '__main__':
    main()
