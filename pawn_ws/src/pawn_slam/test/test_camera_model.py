import numpy as np
import pytest

from pawn_slam.common.camera_model import PinholeCamera

# Plausible Tello-like values (960x720, ~82 deg HFOV), not a real calibration
CAM = PinholeCamera(960, 720, [[920.0, 0, 480.0], [0, 920.0, 360.0], [0, 0, 1]],
                    [-0.02, 0.12, 0.001, -0.0015, -0.2])

CAMERA_INFO_YAML = """
image_width: 960
image_height: 720
camera_name: tello_test
camera_matrix:
  rows: 3
  cols: 3
  data: [920.0, 0.0, 480.0, 0.0, 920.0, 360.0, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.02, 0.12, 0.001, -0.0015, -0.2]
"""


def _image_points(n=200, seed=1):
    rng = np.random.default_rng(seed)
    return np.column_stack([rng.uniform(0, 960, n), rng.uniform(0, 720, n)])


def test_unproject_project_roundtrip():
    px = _image_points()
    rays = CAM.unproject(px)
    assert np.allclose(np.linalg.norm(rays, axis=1), 1) and (rays[:, 2] > 0).all()
    assert np.allclose(CAM.project(rays * 3.7), px, atol=1e-6)    # depth does not matter


def test_center_pixel_is_optical_axis():
    assert np.allclose(CAM.unproject([[480.0, 360.0]]), [[0, 0, 1]])


def test_behind_camera_is_nan():
    px = CAM.project([[0, 0, -1.0], [0.1, 0, 2.0]])
    assert np.isnan(px[0]).all() and np.isfinite(px[1]).all()


def test_matches_opencv():
    cv2 = pytest.importorskip("cv2")
    pts = CAM.unproject(_image_points()) * 2.0
    ref, _ = cv2.projectPoints(pts, np.zeros(3), np.zeros(3), CAM.K, CAM.D)
    assert np.allclose(CAM.project(pts), ref.reshape(-1, 2), atol=1e-6)
    px = _image_points(seed=2)
    ref_n = cv2.undistortPointsIter(px.reshape(-1, 1, 2), CAM.K, CAM.D, np.eye(3), np.eye(3),
                                    (cv2.TERM_CRITERIA_COUNT, 20, 0)).reshape(-1, 2)
    xy = CAM.unproject(px)
    assert np.allclose(xy[:, :2] / xy[:, 2:], ref_n, atol=1e-6)


def test_from_yaml(tmp_path):
    pytest.importorskip("yaml")
    f = tmp_path / "tello_test.yaml"
    f.write_text(CAMERA_INFO_YAML)
    cam = PinholeCamera.from_yaml(f)
    assert cam.name == "tello_test" and (cam.width, cam.height) == (960, 720)
    assert np.allclose(cam.K, CAM.K) and np.allclose(cam.D, CAM.D)
    assert (cam.fx, cam.fy, cam.cx, cam.cy) == (920.0, 920.0, 480.0, 360.0)


def test_rejects_other_distortion_models():
    with pytest.raises(ValueError):
        PinholeCamera.from_camera_info({"image_width": 1, "image_height": 1, "distortion_model": "equidistant",
                                        "camera_matrix": {"data": [1, 0, 0, 0, 1, 0, 0, 0, 1]},
                                        "distortion_coefficients": {"data": [0, 0, 0, 0]}})


def test_short_distortion_vector_is_padded():
    cam = PinholeCamera(10, 10, np.eye(3), [0.1, 0.01, 0, 0])
    assert cam.D.tolist() == [0.1, 0.01, 0, 0, 0]


def test_in_image():
    assert CAM.in_image([[0, 0], [959.5, 719.5], [960, 10], [-1, 5]]).tolist() == [True, True, False, False]
