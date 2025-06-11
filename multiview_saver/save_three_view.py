"""


"""
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import Image, JointState, PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

import time
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from PIL import Image as PILImage
import yaml
import trimesh



CAMERA_TRANSFORM = np.array(
    [[  1.0000,  0.0010,  0.0012, -0.0326],
     [ -0.0010,  1.,     -0.0016, -0.0888],
     [ -0.0010,  0.0016,  1.000,  -0.1404],
     [  0.,      0.,      0.,      1.    ]], dtype=np.float64)

def point_rgb(p):
    x, y, z, rgb = p
    rgb = rgb.view(np.uint32)
    r = (rgb >> 16) & 0x0000ff
    g = (rgb >> 8) & 0x0000ff
    b = (rgb) & 0x0000ff
    return [x, y, z], [r, g, b]


def rgbpcd_to_numpy(rgb_pcd):
    """Concatenate pointcloud position (x,y,z) and color (r,g,b) into a [N,6] matrix.
    """
    #vertices = []
    #cols = []
    #for v, col in rgb_pcd:
    #    vertices.append(v)
    #    cols.append(col)
    vertices, cols = zip(*rgb_pcd)

    vertices = np.stack(vertices).astype(np.float64)
    cols = np.stack(cols).astype(np.float64)
    cols /= 255.

    return np.concatenate([vertices, cols], axis=1)


class ThreeViewSaver(Node):
    def __init__(self):
        super().__init__('three_view_saver')
        self.declare_parameter('num_grippers', 1)

        self._num_grippers = self.get_parameter('num_grippers').get_parameter_value().integer_value

        #self.get_logger().info()

        # placeholders
        self.img = None
        self.colored_pcd = None
        self.joints = None
        self.tcp = None
        self.camera = None
        self.num_saved = 1

        self.create_subscription(Image, '/camera/color/image_raw', self._update_img, 10)
        self.create_subscription(PointCloud2, '/camera/depth_registered/points', self._update_colored_pcd, 10)
        self.create_subscription(PoseStamped, '/tcp_pose_broadcaster/pose', self._update_tcp, 10)
        self.create_subscription(JointState, '/joint_states', self._update_joints, 10)
        self._joint_cmd_pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)

        # Create services
        self.capture_point_service = self.create_service(
            Trigger, 
            '/multiview_saver/capture_point', 
            self.capture_point_service_callback)

        self.restart_count_service = self.create_service(
            Trigger, 
            '/multiview_saver/restart_count', 
            self.restart_count_service_callback)

    def _update_img(self, msg):
        """
        """
        w, h = msg.width, msg.height
        #self.get_logger().info('(w,h) = ({},{})'.format(w, h))
        try:
            rgb_img = np.reshape(msg.data, (h, w, 3))
            #np_arr = np.frombuffer(msg.data, np.uint8)#.reshape((640,480,3))
            #cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        except Exception as e:
            self.get_logger().error('Error processing image: %s' % str(e))

        self.img = rgb_img


    def _update_colored_pcd(self, msg):
        """
        """
        assert isinstance(msg, PointCloud2)
        _pcd = None
        w, h = msg.width, msg.height
        self.get_logger().info('pointcloud: w,h: {},{}'.format(w,h))
        try:
            _fields = ['x', 'y', 'z', 'rgb']
            #_pcd = [point_rgb(p) for p in point_cloud2.read_points(msg, field_names=_fields)]
            point_gen = point_cloud2.read_points(msg, field_names=_fields)
            _pcd = list(map(point_rgb, point_gen))  

        except Exception as e:
            self.get_logger().error('Error processing image: %s' % str(e))

        self.colored_pcd = _pcd


    def _update_tcp(self, msg):
        """
        """
        pos = msg.pose.position
        pos = np.array([pos.x, pos.y, pos.z])
        q = msg.pose.orientation
        r = R.from_quat([q.x, q.y, q.z, q.w])
        euler = r.as_euler('xyz', degrees=True)  # intrinsic rotation
        mat = r.as_matrix()

        tx = np.eye(4)
        tx[:3,:3] = mat
        tx[:3,3] = pos
        cam_pose = tx @ CAMERA_TRANSFORM
        cam_pos = cam_pose[:3,3]
        cam_euler = R.from_matrix(cam_pose[:3,:3]).as_euler('xyz', degrees=True)  # intrinsic rotation

        tcp = {'x': float(pos[0]), 'y': float(pos[1]), 'z': float(pos[2]), \
               'euler_x': float(euler[0]), 'euler_y': float(euler[1]), 'euler_z': float(euler[2])}

        camera = {'x': float(cam_pos[0]), 'y': float(cam_pos[1]), 'z': float(cam_pos[2]), \
                  'euler_x': float(cam_euler[0]), 'euler_y': float(cam_euler[1]), 'euler_z': float(cam_euler[2])}

        for k, v in tcp.items():
            tcp[k] = round(v, 5)

        for k, v in camera.items():
            camera[k] = round(v, 5)

        self.tcp = tcp
        self.camera = camera


    def _update_joints(self, msg):
        """['shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'shoulder_pan_joint']
        -> ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        """
        pos = list(msg.position)
        ret = [pos[-1], *pos[:5]]
        #self.get_logger().info('joint: {}'.format(ret))
        for i in range(6):
            ret[i] = round(ret[i], 5)
        self.joints = ret


    def capture_point_service_callback(self, req: Trigger.Request, resp: Trigger.Response):
        """
        """
        #jt = JointTrajectory()

        #for i in range(3):
            # Pop the latest items in imgs, joints, tcps
        #    self.get_logger().info('loop {}'.format(i))
        #    time.sleep(2)

        # Save TCP pose & camera pose & joints
        start_t = time.time()
        save_dict = {'tcp': [self.tcp], 'camera': [self.camera], 'joints': self.joints}
        with open('robot_state_{}.yaml'.format(self.num_saved), 'w') as yaml_file:
            yaml.dump(save_dict, yaml_file, default_flow_style=False)
        d = time.time() - start_t
        self.get_logger().info('Save robot state file cost: {}'.format(d))
        # Save image
        start_t = time.time()
        image = PILImage.fromarray(self.img)
        image.save('rgb_img_{}.png'.format(self.num_saved))
        d = time.time() - start_t
        self.get_logger().info('Save image cost: {}'.format(d))
        # Save pointcloud
        start_t = time.time()
        colored_pcd = rgbpcd_to_numpy(self.colored_pcd)  # (N, 6) - (x,y,z,r,g,b)
        #scene = trimesh.points.PointCloud(vertices=colored_pcd[:,:3], colors=colored_pcd[:,3:])
        #scene.export(file_obj='pcd_{}.glb'.format(self.num_saved))
        with open('pcd_{}.npy'.format(self.num_saved), 'wb') as f:
            np.save(f, colored_pcd)
        d = time.time() - start_t
        self.get_logger().info('Save pointcloud file cost: {}'.format(d))

        msg = '{}-th point captured. TCP: , joints: , image saved: '.format(self.num_saved)
        self.num_saved += 1

        resp.success = True
        resp.message = msg
        return resp


    def restart_count_service_callback(self, req: Trigger.Request, resp: Trigger.Response):
        """
        """
        msg = 'Restart save count: {} -> 1'.format(self.num_saved)
        self.num_saved = 1
        resp.success = True
        resp.message = msg
        return resp


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ThreeViewSaver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
