"""Camera scan: finds AprilTags (36h11) in each camera and publishes each one as a TF.
In : <cam>/image_raw + <cam>/camera_info for the front and down camera
Out: /tf  camera_optical_frame -> det_<cam>_<id>   (pose of the tag seen by that camera, metres)
     /agv/tag_detections (String JSON: cam, stamp, ids, range)   /agv/<cam>/tags_debug (annotated image)"""
import json
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from .geom import mat_to_tf


class TagDetector(Node):
    def __init__(self):
        super().__init__('tag_detector')
        self.size = self.declare_parameter('tag_size', 0.20).value
        cams = {'front': '/camera', 'down': '/camera_down'}
        self.dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
        self.par = cv2.aruco.DetectorParameters_create()
        self.par.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        h = self.size / 2
        self.obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)
        self.K, self.D = {}, {}
        self.tfb = TransformBroadcaster(self)
        self.pub = self.create_publisher(String, '/agv/tag_detections', 10)
        self.dbg = {}
        for cam, ns in cams.items():
            self.create_subscription(CameraInfo, f'{ns}/camera_info', lambda m, c=cam: self.info(c, m), 1)
            self.create_subscription(Image, f'{ns}/image_raw', lambda m, c=cam: self.image(c, m), 1)
            self.dbg[cam] = self.create_publisher(Image, f'/agv/{cam}/tags_debug', 1)

    def info(self, cam, m):
        self.K[cam] = np.array(m.k, np.float64).reshape(3, 3)
        self.D[cam] = np.array(m.d, np.float64) if len(m.d) else np.zeros(5)

    def image(self, cam, m):
        if cam not in self.K:
            return
        ch = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}.get(m.encoding)
        if ch is None:
            self.get_logger().warn(f'unsupported encoding {m.encoding}', throttle_duration_sec=10)
            return
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, ch)
        gray = img[:, :, 0] if ch == 1 else cv2.cvtColor(img, cv2.COLOR_RGB2GRAY if m.encoding == 'rgb8' else cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dict, parameters=self.par)
        found, tfs = [], []
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                ok, rv, tv, err = cv2.solvePnPGeneric(self.obj, c.reshape(4, 2), self.K[cam], self.D[cam],
                                                       flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok:
                    continue
                T = np.eye(4)
                T[:3, :3] = cv2.Rodrigues(rv[0])[0]
                T[:3, 3] = tv[0].ravel()
                if not np.isfinite(T).all() or T[2, 3] <= 0:        # degenerate pose (tag at the image edge): skip
                    continue
                tfs.append(mat_to_tf(T, m.header.frame_id, f'det_{cam}_{int(i)}', m.header.stamp))
                found.append({'id': int(i), 'range': float(np.linalg.norm(T[:3, 3])), 'err_px': float(err[0])})
        if tfs:
            self.tfb.sendTransform(tfs)
            self.pub.publish(String(data=json.dumps(
                {'cam': cam, 'sec': m.header.stamp.sec, 'nsec': m.header.stamp.nanosec, 'tags': found})))
        if self.dbg[cam].get_subscription_count():
            vis = np.ascontiguousarray(img if ch == 3 else cv2.cvtColor(img, cv2.COLOR_GRAY2RGB))
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(vis, corners, ids)
            out = Image(header=m.header, height=m.height, width=m.width, encoding=m.encoding if ch == 3 else 'rgb8',
                        step=m.width * 3, data=vis.tobytes())
            self.dbg[cam].publish(out)


def main():
    rclpy.init()
    rclpy.spin(TagDetector())
