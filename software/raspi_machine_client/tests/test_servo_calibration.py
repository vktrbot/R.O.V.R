from rovr_common.fake_servo_bus import FakeServoBus
from rovr_common.servo_calibration import ServoCalibration


def test_virtual_calibration_maps_raw_positions() -> None:
    calibration = ServoCalibration(
        servo_id=1,
        minimum=1200,
        maximum=2800,
        home=2048,
        center_raw=3100,
        center_virtual=2048,
    )

    assert calibration.raw_to_command(3100) == 2048
    assert calibration.command_to_raw(2048) == 3100
    assert calibration.command_to_raw(2148) == (3100 + 100) % 4096


def test_fake_bus_loads_home_positions_from_calibration() -> None:
    bus = FakeServoBus(servo_ids=[1], initial_positions=[1500])
    calibration = ServoCalibration(
        servo_id=1,
        minimum=1200,
        maximum=2800,
        home=2048,
        center_raw=3100,
        center_virtual=2048,
    )

    bus.load_calibration([calibration])

    assert bus.get_command_limits() == [(1200, 2800)]
    assert bus.get_home_positions() == [2048]
