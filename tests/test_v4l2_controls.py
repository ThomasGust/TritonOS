import importlib.util
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "video" / "v4l2_controls.py"
SPEC = importlib.util.spec_from_file_location("v4l2_controls_under_test", MODULE_PATH)
v4l2_controls = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
SPEC.loader.exec_module(v4l2_controls)

build_h264_quality_controls = v4l2_controls.build_h264_quality_controls
parse_control_names = v4l2_controls.parse_control_names
set_ctrl_arg = v4l2_controls.set_ctrl_arg


def test_parse_control_names_from_v4l2_ctl_output():
    text = """
Codec Controls

             video_bitrate 0x009909cf (int)    : min=25000 max=16000000 step=25000 default=8000000 value=8000000
       h264_i_frame_period 0x00990a66 (int)    : min=0 max=2147483647 step=1 default=30 value=15
    """

    assert parse_control_names(text) == {"video_bitrate", "h264_i_frame_period"}


def test_build_h264_quality_controls_uses_available_native_controls():
    updates = build_h264_quality_controls(
        {"video_bitrate", "h264_i_frame_period"},
        h264_bitrate=12_000_000,
        h264_gop=15,
    )

    assert updates == {
        "h264_i_frame_period": 15,
        "video_bitrate": 12_000_000,
    }


def test_build_h264_quality_controls_can_be_disabled():
    updates = build_h264_quality_controls(
        {"video_bitrate", "h264_i_frame_period"},
        h264_bitrate=12_000_000,
        h264_gop=15,
        extra={"apply_h264_v4l2_controls": False},
    )

    assert updates == {}


def test_build_h264_quality_controls_allows_explicit_zero_values():
    updates = build_h264_quality_controls(
        {"exposure_dynamic_framerate"},
        h264_bitrate=12_000_000,
        h264_gop=15,
        extra={"v4l2_controls": {"exposure_dynamic_framerate": 0}},
    )

    assert updates == {"exposure_dynamic_framerate": 0}


def test_set_ctrl_arg_is_stable():
    assert set_ctrl_arg({"video_bitrate": 12_000_000, "h264_i_frame_period": 15}) == (
        "h264_i_frame_period=15,video_bitrate=12000000"
    )
