#!/usr/bin/env python3
"""
Main entry point for the ROV.

Hotplug / resiliency goals:
  - Start order doesn't matter (topside can come up first; ROV can come up first)
  - If either side disappears (power cycle, tether unplug, Wi‑Fi drop), the link recovers
    automatically once both are back online
  - Clean shutdown: on SIGINT/SIGTERM we neutralize thrusters and stop pipelines

Services:
  - video RPC server (gst_streamer_rpc)
  - pilot receiver + control loop
  - sensor publisher (including heartbeat)
"""

from __future__ import annotations

import sys
import time
import signal
import threading
import traceback
from typing import Optional, Tuple

import rov_config as cfg


class RestartingThread:
    """Run a long-lived target in a thread; restart it if it crashes."""

    def __init__(self, name: str, target, stop_event: threading.Event, restart_delay_s: float = 1.0):
        self.name = name
        self.target = target
        self.stop_event = stop_event
        self.restart_delay_s = float(restart_delay_s)
        self._thread: Optional[threading.Thread] = None

    def start(self):
        if self._thread and self._thread.is_alive():
            return

        def _runner():
            while not self.stop_event.is_set():
                try:
                    self.target()
                except Exception:
                    print(f"[rov/main] {self.name} crashed; restarting…")
                    traceback.print_exc()
                # backoff before restart (or to allow clean exit)
                for _ in range(int(max(1.0, self.restart_delay_s) * 10)):
                    if self.stop_event.is_set():
                        break
                    time.sleep(0.1)

        self._thread = threading.Thread(target=_runner, daemon=True)
        self._thread.start()

    def join(self, timeout: float = 1.0):
        if self._thread:
            self._thread.join(timeout=timeout)


# --- 1) video --------------------------------------------------------

def start_video_service(stop_event: threading.Event) -> RestartingThread | None:
    """Start gst_streamer RPC in a restartable thread."""
    try:
        from video import gst_streamer_rpc
    except Exception as e:
        print("[rov/main] video: could not import video.gst_streamer_rpc:", e)
        traceback.print_exc()
        return None

    def _run_rpc():
        # Do NOT parse argv here; pass bind explicitly.
        gst_streamer_rpc.start_video_rpc(bind=cfg.VIDEO_RPC_ENDPOINT, stop_event=stop_event)

    rt = RestartingThread("video_rpc", _run_rpc, stop_event=stop_event, restart_delay_s=2.0)
    rt.start()
    print(f"[rov/main] video RPC starting on {cfg.VIDEO_RPC_ENDPOINT}")
    return rt


# --- 2) sensors ------------------------------------------------------

def start_sensor_service(ctrl=None, pilot_rx=None, state=None):
    """
    Create Navigator sensors and start the publisher service.
    Returns the service instance (so it can be stopped cleanly).
    """
    try:
        from sensors.navigator import (
            NavigatorBoard,
            IMUSensor,
            EnvSensor,
            LeakSensor,
            ADCSensor,
            ExternalDepthSensor,
            Bar02Sensor,
            Bar30Sensor,
        )
        from sensors.sensor_pub_service import SensorPublisherService
    except Exception as e:
        print("[rov/main] sensors: could not import sensors modules:", e)
        traceback.print_exc()
        return None

    board = NavigatorBoard()

    sensor_list = [
        IMUSensor(board, rate_hz=20.0),
        EnvSensor(board, rate_hz=2.0),
        LeakSensor(board, rate_hz=2.0),
        ADCSensor(board, rate_hz=5.0),
    ]

    # Heartbeat (lets topside show link + armed state + pilot freshness)
    try:
        from sensors.heartbeat import HeartbeatSensor

        def _hb_state():
            armed = bool(state.is_armed()) if state is not None else False
            seq = None
            age = None
            connected = None
            if pilot_rx is not None:
                p, a = pilot_rx.get_latest()
                age = float(a) if a is not None else None
                seq = int(p.seq) if p is not None else None
                try:
                    cs = pilot_rx.connection_snapshot()
                    connected = bool(cs.get("connected", False))
                except Exception:
                    connected = None
            return {
                "armed": armed,
                "pilot_age": age,
                "pilot_seq": seq,
                "topside_connected": connected,
            }

        sensor_list.append(HeartbeatSensor(state_fn=_hb_state, rate_hz=1.0))
    except Exception as e:
        if getattr(cfg, "DEBUG", False):
            print("[rov/main] heartbeat disabled:", e)

    # External depth sensor (Blue Robotics MS5837: Bar30 / Bar02)
    use_external = bool(getattr(cfg, "USE_EXTERNAL_DEPTH", False))
    use_bar02 = bool(getattr(cfg, "USE_BAR02", False))
    use_bar30 = bool(getattr(cfg, "USE_BAR30", False))

    if use_external or use_bar02 or use_bar30:
        try:
            def _get_buses(prefix: str, default_bus: int = 6):
                buses = getattr(cfg, f"{prefix}_I2C_BUSES", None)
                if buses is not None:
                    return buses
                return int(getattr(cfg, f"{prefix}_I2C_BUS", getattr(cfg, "BAR30_I2C_BUS", default_bus)))

            if use_bar02:
                buses = _get_buses("BAR02")
                sensor_list.append(
                    Bar02Sensor(
                        rate_hz=float(getattr(cfg, "BAR02_RATE_HZ", getattr(cfg, "BAR30_RATE_HZ", 5.0))),
                        bus=buses,
                        model=getattr(cfg, "BAR02_MODEL", "02BA"),
                        fluid_density=float(getattr(cfg, "BAR02_FLUID_DENSITY", getattr(cfg, "BAR30_FLUID_DENSITY", 1029))),
                        osr=int(getattr(cfg, "BAR02_OSR", getattr(cfg, "BAR30_OSR", 5))),
                        surface_cal_samples=int(getattr(cfg, "BAR02_SURFACE_CAL_SAMPLES", getattr(cfg, "BAR30_SURFACE_CAL_SAMPLES", 15))),
                        surface_cal_delay_s=float(getattr(cfg, "BAR02_SURFACE_CAL_DELAY_S", getattr(cfg, "BAR30_SURFACE_CAL_DELAY_S", 0.02))),
                    )
                )
            elif use_external:
                buses = getattr(cfg, "EXTERNAL_DEPTH_I2C_BUSES", None)
                if buses is None:
                    buses = _get_buses("BAR30")
                sensor_list.append(
                    ExternalDepthSensor(
                        rate_hz=float(getattr(cfg, "EXTERNAL_DEPTH_RATE_HZ", getattr(cfg, "BAR30_RATE_HZ", 5.0))),
                        bus=buses,
                        model=getattr(cfg, "EXTERNAL_DEPTH_MODEL", getattr(cfg, "BAR30_MODEL", "auto")),
                        fluid_density=float(getattr(cfg, "EXTERNAL_DEPTH_FLUID_DENSITY", getattr(cfg, "BAR30_FLUID_DENSITY", 1029))),
                        osr=int(getattr(cfg, "EXTERNAL_DEPTH_OSR", getattr(cfg, "BAR30_OSR", 5))),
                        surface_cal_samples=int(getattr(cfg, "EXTERNAL_DEPTH_SURFACE_CAL_SAMPLES", getattr(cfg, "BAR30_SURFACE_CAL_SAMPLES", 15))),
                        surface_cal_delay_s=float(getattr(cfg, "EXTERNAL_DEPTH_SURFACE_CAL_DELAY_S", getattr(cfg, "BAR30_SURFACE_CAL_DELAY_S", 0.02))),
                    )
                )
            else:
                buses = _get_buses("BAR30")
                sensor_list.append(
                    Bar30Sensor(
                        rate_hz=float(getattr(cfg, "BAR30_RATE_HZ", 5.0)),
                        bus=buses,
                        model=getattr(cfg, "BAR30_MODEL", "auto"),
                        fluid_density=float(getattr(cfg, "BAR30_FLUID_DENSITY", 1029)),
                        osr=int(getattr(cfg, "BAR30_OSR", 5)),
                        surface_cal_samples=int(getattr(cfg, "BAR30_SURFACE_CAL_SAMPLES", 15)),
                        surface_cal_delay_s=float(getattr(cfg, "BAR30_SURFACE_CAL_DELAY_S", 0.02)),
                    )
                )
        except Exception as e:
            print("[rov/main] sensors: external depth enabled but failed to init:", e)

    # Network stats sensor (tether/wifi visibility + throughput)
    if getattr(cfg, "NET_STATS_ENABLE", False):
        try:
            from sensors.network import NetworkStatsSensor

            sensor_list.append(
                NetworkStatsSensor(
                    rate_hz=float(getattr(cfg, "NET_STATS_RATE_HZ", 1.0)),
                    iface=getattr(cfg, "NET_STATS_IFACE", None),
                )
            )
        except Exception as e:
            print("[rov/main] network stats sensor disabled:", e)

    srv = SensorPublisherService(
        bind_endpoint=cfg.SENSOR_PUB_ENDPOINT,
        sensors=sensor_list,
        debug=getattr(cfg, "DEBUG", False),
    )
    srv.start()
    print(f"[rov/main] sensor PUB started on {cfg.SENSOR_PUB_ENDPOINT}")
    return srv


# --- 3) control / pilot ----------------------------------------------

def start_control_service():
    """
    Start the pilot SUB + control loop, attach hardware sink if available.
    Returns (ctrl, pilot_rx, state).
    """
    try:
        from control.pilot_receiver import PilotReceiver
        from control.control_service import ControlService, ControlGains, ROVControlState
    except Exception as e:
        print("[rov/main] control: could not import control modules:", e)
        traceback.print_exc()
        return None, None, None

    # optional: hardware sink
    hw_sink = None
    try:
        from motion import pwm
        from motion.channel_map import ChannelMap
    except Exception as e:
        print("[rov/main] motion: not using hardware PWM (import failed):", e)
    else:
        try:
            if hasattr(pwm, "ThrustWriter"):
                chanmap = ChannelMap.from_config(cfg)

                thrust_cfg = pwm.ThrustConfig(
                    freq_hz=getattr(cfg, "PWM_FREQ_HZ", 50.0),
                    neutral_us=getattr(cfg, "PWM_NEUTRAL_US", 1500),
                    span_us=getattr(cfg, "PWM_SPAN_US", 400),
                    min_us=getattr(cfg, "PWM_MIN_US", 1100),
                    max_us=getattr(cfg, "PWM_MAX_US", 1900),
                    deadband_norm=getattr(cfg, "PWM_DEADBAND", 0.07),
                    deadband_us=getattr(cfg, "PWM_DEADBAND_US", 25),
                    trim_us=getattr(cfg, "PWM_TRIM_US", 0),
                    esc_init_hold_s=getattr(cfg, "ESC_INIT_HOLD_S", 3.0),
                    hardware_arm_disarm=bool(getattr(cfg, "HARDWARE_ARM_DISARM", True)),
                    pwm_rearm_off_s=float(getattr(cfg, "PWM_REARM_OFF_S", 0.35)),
                    pwm_disarm_hold_s=float(getattr(cfg, "PWM_DISARM_HOLD_S", 0.25)),
                    disable_pwm_on_disarm=bool(getattr(cfg, "DISABLE_PWM_ON_DISARM", True)),
                    keep_pwm_enabled_on_disarm=getattr(cfg, "KEEP_PWM_ENABLED_ON_DISARM", True),
                    channel_base=getattr(cfg, "PWM_CHANNEL_BASE", "auto"),
                )

                rev_map = {}
                rev_map.update(getattr(cfg, "THRUSTER_REVERSED", {}) or {})
                rev_map.update(getattr(cfg, "CHANNEL_REVERSED", {}) or {})

                aux_channels = {}
                aux_cfg = {}
                if hasattr(cfg, "LIGHTS_PWM_CHANNEL") and getattr(cfg, "LIGHTS_PWM_CHANNEL") is not None:
                    try:
                        aux_channels["lights"] = int(getattr(cfg, "LIGHTS_PWM_CHANNEL"))
                        aux_cfg["lights"] = pwm.AuxOutputConfig(
                            min_us=int(getattr(cfg, "LIGHTS_US_MIN", getattr(cfg, "LIGHTS_MIN_US", 1100))),
                            max_us=int(getattr(cfg, "LIGHTS_US_MAX", getattr(cfg, "LIGHTS_MAX_US", 1900))),
                            off_us=int(getattr(cfg, "LIGHTS_US_OFF", getattr(cfg, "LIGHTS_OFF_US", 1100))),
                            deadband_norm=float(getattr(cfg, "LIGHTS_DEADBAND_NORM", getattr(cfg, "LIGHTS_DEADZONE", 0.02))),
                            trim_us=int(getattr(cfg, "LIGHTS_TRIM_US", 0)),
                            allow_when_disarmed=bool(getattr(cfg, "LIGHTS_ALLOW_WHEN_DISARMED", True)),
                            force_off_on_disarm=bool(getattr(cfg, "LIGHTS_FORCE_OFF_ON_DISARM", False)),
                        )
                    except Exception:
                        aux_channels = {}
                        aux_cfg = {}

                hw_sink = pwm.ThrustWriter(
                    thruster_channels=chanmap.thrusters,
                    cfg=thrust_cfg,
                    reversed_map=rev_map if rev_map else None,
                    aux_channels=aux_channels if aux_channels else None,
                    aux_cfg=aux_cfg if aux_cfg else None,
                    debug=getattr(cfg, "DEBUG", False),
                    auto_enable=bool(getattr(cfg, "PWM_AUTO_ENABLE", False)),
                )
                print("[rov/main] motion: using Navigator PWM via bluerobotics_navigator")
            elif hasattr(pwm, "write_thrust"):
                hw_sink = pwm.write_thrust
                print("[rov/main] motion: using pwm.write_thrust(...)")
            else:
                print("[rov/main] motion: pwm.py imported but no known sink found; will print thrusters")
        except Exception as e:
            print("[rov/main] motion: hardware PWM init failed:", e)
            hw_sink = None

    if hw_sink is None:
        print("[rov/main] motion: NO hardware PWM sink -> motors will NOT run (dry_run mode).")
        if bool(getattr(cfg, "REQUIRE_HARDWARE_PWM", False)):
            raise SystemExit("[rov/main] FATAL: REQUIRE_HARDWARE_PWM is set, but hardware PWM could not be initialized.")
    else:
        print("[rov/main] motion: hardware PWM sink ready.")

    pilot_rx = PilotReceiver(bind_endpoint=cfg.PILOT_SUB_ENDPOINT, debug=getattr(cfg, "PILOT_RX_DEBUG", False))
    pilot_rx.start()

    state = ROVControlState()
    state.set_armed(False)

    gains = ControlGains(
        surge=1.0,
        sway=1.0,
        heave=1.0,
        yaw=1.0,
        pitch=1.0,
        roll=1.0,
        power_scale=float(getattr(cfg, "POWER_SCALE", 1.0)),
    )
    ctrl = ControlService(
        pilot_rx=pilot_rx,
        gains=gains,
        control_state=state,
        rate_hz=float(getattr(cfg, "CONTROL_RATE_HZ", 50.0)),
        ttl=float(getattr(cfg, "PILOT_TTL", 0.5)),
        debug=bool(getattr(cfg, "CONTROL_DEBUG", False)),
        dry_run=(hw_sink is None),
    )

    if hw_sink is not None and hasattr(ctrl, "set_hw_sink"):
        try:
            ctrl.set_hw_sink(hw_sink)
            ctrl.dry_run = False
        except Exception as e:
            print("[rov/main] warning: could not attach hw sink:", e)

    ctrl.start()
    print(f"[rov/main] control loop started (rate={getattr(cfg, 'CONTROL_RATE_HZ', 50.0)} Hz)")
    return ctrl, pilot_rx, state


def main():
    print("[rov/main] starting services…")

    stop_event = threading.Event()

    def _request_stop(signum=None, frame=None):
        stop_event.set()

    # SIGINT/SIGTERM => clean shutdown
    try:
        signal.signal(signal.SIGINT, _request_stop)
        signal.signal(signal.SIGTERM, _request_stop)
    except Exception:
        pass

    # Optional: start netdiag server (UDP echo + TCP throughput)
    if getattr(cfg, "NETDIAG_ENABLE", False):
        try:
            from tools import netdiag_server

            netdiag_server.start_in_thread(
                bind_host=str(getattr(cfg, "NETDIAG_BIND_HOST", "0.0.0.0")),
                port=int(getattr(cfg, "NETDIAG_PORT", 7700)),
                verbose=bool(getattr(cfg, "DEBUG", False)),
            )
            print(
                f"[rov/main] netdiag server started on {getattr(cfg, 'NETDIAG_BIND_HOST', '0.0.0.0')}:{getattr(cfg, 'NETDIAG_PORT', 7700)}"
            )
        except Exception as e:
            print("[rov/main] netdiag server disabled:", e)

    video_thread = start_video_service(stop_event)
    ctrl, pilot_rx, state = start_control_service()
    sensor_srv = start_sensor_service(ctrl=ctrl, pilot_rx=pilot_rx, state=state)

    print("[rov/main] all services started.")
    try:
        while not stop_event.is_set():
            time.sleep(0.5)
    finally:
        print("[rov/main] shutting down…")

        # Stop in the safest order: disarm first via control loop.
        try:
            if ctrl is not None:
                ctrl.stop()
        except Exception:
            pass

        try:
            if pilot_rx is not None:
                pilot_rx.stop()
        except Exception:
            pass

        try:
            if sensor_srv is not None:
                sensor_srv.stop()
        except Exception:
            pass

        # Stop video RPC (and allow the restarting thread to exit)
        stop_event.set()
        try:
            if video_thread is not None:
                video_thread.join(timeout=2.0)
        except Exception:
            pass

        print("[rov/main] shutdown complete.")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
