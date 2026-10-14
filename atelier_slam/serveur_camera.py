#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import socket
import threading
import time


class ROSRawStreamServer(Node):
    """
    ROS node that streams raw BGR frames via TCP.
    Subscribes to /camera/image_raw and serves raw BGR24 data.
    """

    def __init__(self, port=9999):
        super().__init__('ros_raw_stream_server')

        self.port = port
        self.bridge = CvBridge()
        self.current_frame = None
        self.frame_lock = threading.Lock()
        self.running = False
        self.frame_count = 0
        self.start_time = time.time()

        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        self.get_logger().info(f"[RAW STREAM] Souscrit à /camera/image_raw sur port {self.port}")

    def camera_callback(self, msg):
        """Callback pour chaque frame caméra"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            with self.frame_lock:
                self.current_frame = frame

            self.frame_count += 1

            # Afficher FPS toutes les frames (ou moins souvent)
            if self.frame_count == 1:
                self.get_logger().info(f"[RAW STREAM] ✓ Premier frame reçu! {frame.shape}")

            if self.frame_count % 30 == 0:
                elapsed = time.time() - self.start_time
                fps = self.frame_count / elapsed
                self.get_logger().info(f"[RAW STREAM] FPS: {fps:.1f} | Frames: {self.frame_count}")

        except Exception as e:
            self.get_logger().error(f"[RAW STREAM] Erreur conversion: {e}")

    def start_server(self):
        """Démarrer le serveur TCP de flux brut"""
        self.running = True

        # Thread serveur TCP
        server_thread = threading.Thread(target=self._run_server, daemon=True)
        server_thread.start()

        self.get_logger().info(f"[RAW STREAM] ✓ Serveur prêt sur tcp://0.0.0.0:{self.port}")

    def _run_server(self):
        """Serveur TCP pour flux brut"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('0.0.0.0', self.port))
            sock.listen(5)

            self.get_logger().info(f"[RAW STREAM] Serveur en écoute sur port {self.port}")

            while self.running:
                try:
                    client_sock, addr = sock.accept()
                    self.get_logger().info(f"[RAW STREAM] Client connecté: {addr}")

                    client_thread = threading.Thread(
                        target=self._handle_client,
                        args=(client_sock, addr),
                        daemon=True
                    )
                    client_thread.start()

                except Exception as e:
                    if self.running:
                        self.get_logger().error(f"[RAW STREAM] Erreur accept: {e}")

            sock.close()

        except Exception as e:
            self.get_logger().error(f"[RAW STREAM] Erreur serveur: {e}")

    def _handle_client(self, client_sock, addr):
        """Envoyer les frames brutes BGR au client"""
        try:
            frames_sent = 0
            while self.running:
                try:
                    with self.frame_lock:
                        frame = self.current_frame

                    if frame is not None:
                        # Envoyer le frame brut en BGR24
                        raw_data = frame.tobytes()
                        try:
                            client_sock.sendall(raw_data)
                            frames_sent += 1
                        except:
                            break
                    else:
                        time.sleep(0.01)

                except Exception as e:
                    if frames_sent > 0:
                        self.get_logger().error(f"[RAW STREAM] Erreur envoi: {e}")
                    break

        except Exception as e:
            self.get_logger().error(f"[RAW STREAM] Erreur client {addr}: {e}")

        finally:
            try:
                client_sock.close()
            except:
                pass

            self.get_logger().info(f"[RAW STREAM] Client déconnecté: {addr} ({frames_sent} frames)")


def main(args=None):
    rclpy.init(args=args)

    server = ROSRawStreamServer(port=9999)
    server.start_server()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.running = False
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
