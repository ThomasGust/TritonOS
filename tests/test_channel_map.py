from types import SimpleNamespace

from motion.channel_map import ChannelMap


def test_channel_map_falls_back_to_repo_defaults_when_config_is_sparse():
    cfg = SimpleNamespace()
    cm = ChannelMap.from_config(cfg)

    assert cm.thrusters == {
        "H_FL": 8,
        "H_FR": 6,
        "H_RL": 7,
        "H_RR": 2,
        "V_FL": 3,
        "V_FR": 4,
        "V_RL": 9,
        "V_RR": 1,
    }
    assert cm.aux["lights"] == 5
    assert cm.aux["wrist_rotate"] == 10
