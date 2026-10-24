"""Check expected shutdown paths without suppressing playback errors."""

from unittest.mock import Mock

from pawn_slam import euroc_camera_publisher as publisher
import pytest
from rclpy.executors import ExternalShutdownException


@pytest.mark.parametrize('error', [None, KeyboardInterrupt(), ExternalShutdownException()])
def test_expected_shutdown(monkeypatch, error):
    """Clean up normally when spin returns or a shutdown signal interrupts it."""
    node = Mock()
    init = Mock()
    shutdown = Mock()
    monkeypatch.setattr(publisher, 'EurocCameraPublisher', lambda: node)
    monkeypatch.setattr(publisher.rclpy, 'init', init)
    monkeypatch.setattr(publisher.rclpy, 'spin', Mock(side_effect=error))
    monkeypatch.setattr(publisher.rclpy, 'try_shutdown', shutdown)

    publisher.main()

    init.assert_called_once_with(args=None)
    node.destroy_node.assert_called_once_with()
    shutdown.assert_called_once_with()


def test_playback_error_propagates(monkeypatch):
    """Keep unrelated runtime errors visible while still cleaning up."""
    node = Mock()
    shutdown = Mock()
    monkeypatch.setattr(publisher, 'EurocCameraPublisher', lambda: node)
    monkeypatch.setattr(publisher.rclpy, 'init', Mock())
    monkeypatch.setattr(publisher.rclpy, 'spin', Mock(side_effect=ValueError('bad frame')))
    monkeypatch.setattr(publisher.rclpy, 'try_shutdown', shutdown)

    with pytest.raises(ValueError, match='bad frame'):
        publisher.main()

    node.destroy_node.assert_called_once_with()
    shutdown.assert_called_once_with()
