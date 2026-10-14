"""Import + unit tests for rak_imu_driver (no hardware access)."""

import importlib


def test_module_imports():
    mod = importlib.import_module("rak_imu_driver.spi_imu")
    assert hasattr(mod, "main")
    # Scale constants must match the datasheet values the node claims to use.
    assert abs(mod.ACCEL_SCALE - 0.000598205) < 1e-9
    assert abs(mod.GYRO_SCALE - 0.000152716) < 1e-9
    assert 1660 in mod.ODR_BITS


def test_spi_transfer_struct_layout():
    """spi_ioc_transfer must stay 32 bytes or the ioctl number is wrong."""
    import ctypes

    from rak_imu_driver.spi_imu import SpiIocTransfer

    assert ctypes.sizeof(SpiIocTransfer) == 32
