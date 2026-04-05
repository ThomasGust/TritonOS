from control.control_service import ControlService, ROVControlState


def test_send_to_hw_warns_once_in_dry_run_when_armed_with_nonzero_output(capsys):
    svc = object.__new__(ControlService)
    svc.dry_run = True
    svc.state = ROVControlState()
    svc.state.set_armed(True)
    svc._warned_dry_run = False
    svc._warned_no_sink = False
    svc._warned_sink_disarmed = False
    svc._hw_sink = None
    svc._sink_armed = False

    ControlService._send_to_hw(svc, {"H_FL": 0.25, "H_FR": 0.0})
    first = capsys.readouterr()
    assert "dry_run=True" in first.out
    assert svc._warned_dry_run is True

    ControlService._send_to_hw(svc, {"H_FL": 0.25, "H_FR": 0.0})
    second = capsys.readouterr()
    assert second.out == ""
