#!/usr/bin/env python3
"""
color_detector_node.py  —  robotics_subject / JetAuto

Detecta cores (vermelho, verde, azul) nas paredes do labirinto usando a
câmera RGB integrada ao robô.

Comportamento:
  • Vermelho  → publica comando LEFT  (/color_command)
  • Verde     → publica comando RIGHT (/color_command)
  • Azul      → publica comando NONE  (/color_command)

Deduplicação por posição da PAREDE:
  • A posição da parede é estimada projetando a leitura frontal do lidar
    a partir da pose do robô no momento da detecção.
  • O comando é publicado APENAS UMA VEZ por parede nova — identificada
    pelo ponto estimado da parede, não pela posição do robô. Isso garante
    que ver a mesma parede pela frente ou pela trás seja corretamente
    ignorado, e que duas paredes da mesma cor distantes sejam tratadas
    como eventos distintos.

Tópicos publicados:
  /color_command               std_msgs/String  ("LEFT:wx:wy" | "RIGHT:wx:wy" | "NONE")
  /color_detection_markers     visualization_msgs/MarkerArray
  /jetauto/camera/color_debug  sensor_msgs/Image  (imagem com máscara overlay)

Tópicos subscritos:
  /jetauto/camera/rgb_camera_sensor/image_raw  sensor_msgs/Image
  /odometry/filtered                           nav_msgs/Odometry
  /jetauto/lidar/scan                          sensor_msgs/LaserScan
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Time as RosTime

import cv2
import numpy as np
from cv_bridge import CvBridge


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def range_at(ranges, angle_min: float, angle_inc: float,
             deg: float, window_deg: float = 15.0, max_r: float = 3.0) -> float:
    """Retorna a menor leitura válida do lidar dentro da janela centrada em deg°."""
    rad = math.radians(deg)
    hw  = math.radians(window_deg)
    vals = [r for i, r in enumerate(ranges)
            if abs(angle_min + i * angle_inc - rad) <= hw
            and math.isfinite(r) and r > 0.05]
    return min(vals) if vals else max_r


# ──────────────────────────────────────────────────────────────────────────────
# Node
# ──────────────────────────────────────────────────────────────────────────────

class ColorDetectorNode(Node):
    """Detecta cores nas paredes e publica comandos de direção."""

    # Raio (m) para considerar duas detecções como a mesma parede.
    # Compara a posição ESTIMADA DA PAREDE (via lidar), não do robô.
    WALL_DEDUPE_DIST = 2.0

    # Raio ampliado usado quando o robô está olhando na direção oposta (~180°)
    # em relação à detecção original — típico de ver a mesma parede pelo outro lado.
    # Deve cobrir: espessura da parede + erro de projeção lidar nos dois sentidos.
    WALL_DEDUPE_DIST_BACK = 3.0

    # Distância máxima de projeção usada como fallback quando o lidar
    # não retorna leitura válida à frente.
    WALL_MAX_PROJ = 1.925

    # Fração mínima dos pixels da imagem para considerar detecção válida.
    # Valor mais alto = robô precisa estar mais perto da parede para detectar.
    # 0.04 detectava de longe; 0.12 exige que a cor ocupe ~12% do frame.
    MIN_COLOR_RATIO = 0.06

    # FOV horizontal da câmera RGB (rad) — definido no depth_camera.urdf.xacro
    CAMERA_HFOV = 1.3962634   # 80°

    # Largura da imagem em pixels (deve bater com o urdf)
    IMAGE_WIDTH = 640

    # Mapeamento de cor → (texto ação, emoji, rgba_rviz, comando)
    COLOR_INFO = {
        'RED':   ('ESQUERDA', '🔴', (1.0, 0.15, 0.15, 1.0), 'LEFT'),
        'GREEN': ('DIREITA',  '🟢', (0.15, 1.0, 0.15, 1.0), 'RIGHT'),
        'BLUE':  ('NENHUMA',  '🔵', (0.15, 0.15, 1.0, 1.0), 'NONE'),
    }

    def __init__(self):
        super().__init__('color_detector')

        self.declare_parameter('wall_dedupe_dist',      self.WALL_DEDUPE_DIST)
        self.declare_parameter('wall_dedupe_dist_back', self.WALL_DEDUPE_DIST_BACK)
        self.declare_parameter('wall_max_proj',         self.WALL_MAX_PROJ)
        self.declare_parameter('min_color_ratio',       self.MIN_COLOR_RATIO)

        self.WALL_DEDUPE_DIST      = self.get_parameter('wall_dedupe_dist').value
        self.WALL_DEDUPE_DIST_BACK = self.get_parameter('wall_dedupe_dist_back').value
        self.WALL_MAX_PROJ         = self.get_parameter('wall_max_proj').value
        self.MIN_COLOR_RATIO       = self.get_parameter('min_color_ratio').value

        self.bridge        = CvBridge()
        self.current_pose  = None   # (x, y, yaw)
        self.latest_scan   = None   # sensor_msgs/LaserScan
        self._prev_color   = None   # última cor detectada (evita spam de frame)

        # Lista de paredes já vistas: (wall_x, wall_y, wall_yaw, color)
        # wall_x/y: posição estimada da parede no mapa (via lidar).
        # wall_yaw: heading do robô quando detectou — define a "face" da parede.
        self._seen_walls = []

        # ── Publishers ──────────────────────────────────────────────────────
        self.cmd_pub    = self.create_publisher(String,      '/color_command',              10)
        self.marker_pub = self.create_publisher(MarkerArray, '/color_detection_markers',    10)
        self.debug_pub  = self.create_publisher(Image,       '/jetauto/camera/color_debug', 10)
        self._marker_id = 0

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(Image,     '/jetauto/camera/rgb_camera_sensor/image_raw',
                                 self._image_cb, 10)
        self.create_subscription(Odometry,  '/odometry/filtered',   self._odom_cb,  10)
        self.create_subscription(LaserScan, '/jetauto/lidar/scan',  self._scan_cb,  10)

        self.get_logger().info(
            'ColorDetector iniciado — aguardando câmera e lidar...'
        )

    # ── Odometry ─────────────────────────────────────────────────────────────

    def _odom_cb(self, msg):
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quat(msg.pose.pose.orientation),
        )

    # ── Lidar ────────────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    # ── Processamento de imagem ───────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        if self.current_pose is None:
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge error: {exc}')
            return

        color, masks, centroid_x = self._detect_dominant_color(bgr)

        # ── Imagem de debug com overlay colorido ─────────────────────────────
        self._publish_debug_image(bgr, masks, color)

        # ── Sem cor detectada: reseta transição de frame ──────────────────────
        if color is None:
            self._prev_color = None
            return

        # ── Transição de frame: só age na primeira vez que a cor aparece ──────
        # Evita publicar a cada frame enquanto a parede está visível.
        if color == self._prev_color:
            return
        self._prev_color = color

        # ── Ângulo do lidar correspondente ao centroide da cor na imagem ──────
        # centroid_x normalizado (-0.5 esq … +0.5 dir) → ângulo em rad.
        # No ROS, lidar positivo = esquerda do robô, negativo = direita.
        # A câmera tem x crescendo para a direita, então invertemos o sinal.
        lidar_deg = 0.0
        if centroid_x is not None:
            lidar_rad = -centroid_x * self.CAMERA_HFOV   # rad, sinal corrigido
            lidar_deg = math.degrees(lidar_rad)

        # ── Estimar posição da parede no mapa via lidar ───────────────────────
        rx, ry, ryaw = self.current_pose
        if self.latest_scan is not None:
            scan = self.latest_scan
            dist = range_at(scan.ranges, scan.angle_min, scan.angle_increment,
                            lidar_deg, window_deg=8.0, max_r=self.WALL_MAX_PROJ)
            dist = min(dist, self.WALL_MAX_PROJ)
        else:
            dist = self.WALL_MAX_PROJ

        # O ângulo real no mapa = yaw do robô + ângulo lidar
        wall_angle = ryaw + math.radians(lidar_deg)
        wall_x = rx + dist * math.cos(wall_angle)
        wall_y = ry + dist * math.sin(wall_angle)

        self.get_logger().debug(
            f'[{color}] centroid_x={centroid_x:.2f} → lidar_deg={lidar_deg:.1f}° '
            f'dist={dist:.2f}m → parede=({wall_x:.2f},{wall_y:.2f})'
        )

        # ── Deduplicação por posição da PAREDE com consciência de ângulo ──────
        #
        # Dois cenários para "mesma parede":
        #   A) Mesmo lado  — robô olha na mesma direção (|Δyaw| < 90°).
        #      Raio pequeno (WALL_DEDUPE_DIST) é suficiente.
        #   B) Lado oposto — robô olha ~180° diferente (|Δyaw| > 90°).
        #      O ponto projetado cai do outro lado da parede, podendo estar
        #      2-3 m longe do ponto original. Usa raio maior (WALL_DEDUPE_DIST_BACK).
        already = False
        for wx, wy, wyaw, wc in self._seen_walls:
            if wc != color:
                continue
            dist_to_known = math.hypot(wall_x - wx, wall_y - wy)
            angle_diff = abs(math.atan2(math.sin(ryaw - wyaw),
                                        math.cos(ryaw - wyaw)))
            # Lado oposto: diferença angular > 90°
            threshold = (self.WALL_DEDUPE_DIST_BACK
                         if angle_diff > math.pi / 2
                         else self.WALL_DEDUPE_DIST)
            if dist_to_known < threshold:
                already = True
                self.get_logger().debug(
                    f'[{color}] parede em ({wall_x:.2f},{wall_y:.2f}) '
                    f'já vista em ({wx:.2f},{wy:.2f}) '
                    f'Δyaw={math.degrees(angle_diff):.0f}° thresh={threshold:.1f}m — ignorando.'
                )
                break

        if already:
            return

        # ── Nova parede: registra, anuncia e envia comando ────────────────────
        self._seen_walls.append((wall_x, wall_y, ryaw, color))
        self._announce(color, wall_x, wall_y)
        self._send_command(color, wall_x, wall_y)

    # ── Detecção de cor por HSV ───────────────────────────────────────────────

    def _detect_dominant_color(self, bgr: np.ndarray):
        """
        Retorna (cor_dominante, dicionário_de_máscaras, centroid_x).
        centroid_x é a coluna normalizada do centroide da cor detectada
        em relação ao centro da imagem: -0.5 = extrema esquerda, +0.5 = extrema direita.
        Retorna None para centroid_x se nenhuma cor for detectada.
        Usa espaço HSV para robustez a variações de iluminação.
        """
        hsv   = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        total = bgr.shape[0] * bgr.shape[1]

        # Vermelho: engloba 0-10° e 170-180° (hue circular)
        m_red1 = cv2.inRange(hsv, np.array([0,   120, 80]),  np.array([10,  255, 255]))
        m_red2 = cv2.inRange(hsv, np.array([168, 120, 80]),  np.array([180, 255, 255]))
        m_red  = cv2.bitwise_or(m_red1, m_red2)

        # Verde: 40-85° de hue
        m_green = cv2.inRange(hsv, np.array([40, 90, 80]),  np.array([85, 255, 255]))

        # Azul: 100-135° de hue
        m_blue  = cv2.inRange(hsv, np.array([100, 90, 80]), np.array([135, 255, 255]))

        # Suavização morfológica para remover ruído
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        m_red   = cv2.morphologyEx(m_red,   cv2.MORPH_OPEN, kernel)
        m_green = cv2.morphologyEx(m_green, cv2.MORPH_OPEN, kernel)
        m_blue  = cv2.morphologyEx(m_blue,  cv2.MORPH_OPEN, kernel)

        ratios = {
            'RED':   cv2.countNonZero(m_red)   / total,
            'GREEN': cv2.countNonZero(m_green) / total,
            'BLUE':  cv2.countNonZero(m_blue)  / total,
        }
        masks = {'RED': m_red, 'GREEN': m_green, 'BLUE': m_blue}

        best = max(ratios, key=ratios.get)
        if ratios[best] >= self.MIN_COLOR_RATIO:
            # Calcula centroide X da máscara dominante
            M = cv2.moments(masks[best])
            if M['m00'] > 0:
                cx_px = M['m10'] / M['m00']
            else:
                cx_px = bgr.shape[1] / 2.0
            # Normaliza: 0.0 = centro, -0.5 = esquerda, +0.5 = direita
            centroid_x = (cx_px - bgr.shape[1] / 2.0) / bgr.shape[1]
            return best, masks, centroid_x
        return None, masks, None

    # ── Imagem debug ──────────────────────────────────────────────────────────

    def _publish_debug_image(self, bgr: np.ndarray, masks: dict, detected_color):
        """Publica imagem com máscara, centroide e ângulo lidar estimado."""
        overlay = bgr.copy()
        h, w = bgr.shape[:2]

        color_bgr_map = {
            'RED':   (0,   0,   255),
            'GREEN': (0,   255, 0),
            'BLUE':  (255, 0,   0),
        }

        # Linha central de referência
        cv2.line(overlay, (w // 2, 0), (w // 2, h), (80, 80, 80), 1)

        if detected_color is not None and detected_color in masks:
            mask = masks[detected_color]
            tint = np.zeros_like(bgr)
            tint[mask > 0] = color_bgr_map[detected_color]
            cv2.addWeighted(tint, 0.5, overlay, 0.5, 0, overlay)

            info  = self.COLOR_INFO[detected_color]
            clr   = color_bgr_map[detected_color]
            label = f'{info[1]} {detected_color} -> {info[0]}'
            cv2.putText(overlay, label, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, clr, 2)

            # Centroide + ângulo lidar
            M = cv2.moments(mask)
            if M['m00'] > 0:
                cx_px  = int(M['m10'] / M['m00'])
                cy_px  = int(M['m01'] / M['m00'])
                norm_x = (cx_px - w / 2.0) / w
                lidar_deg = -norm_x * math.degrees(self.CAMERA_HFOV)
                # Cruz no centroide
                cv2.drawMarker(overlay, (cx_px, cy_px), clr,
                               cv2.MARKER_CROSS, 20, 2)
                # Linha do centro até o centroide
                cv2.line(overlay, (w // 2, cy_px), (cx_px, cy_px), clr, 1)
                cv2.putText(overlay,
                            f'lidar {lidar_deg:+.1f}deg',
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, clr, 2)
        else:
            cv2.putText(overlay, 'Sem cor', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(overlay, 'bgr8')
            self.debug_pub.publish(debug_msg)
        except Exception:
            pass

    # ── Publicar comando de giro ──────────────────────────────────────────────

    def _send_command(self, color: str, wall_x: float, wall_y: float):
        """Publica 'DIRECTION:wall_x:wall_y' para o maze_solver."""
        cmd  = self.COLOR_INFO[color][3]   # 'LEFT' | 'RIGHT' | 'NONE'
        msg  = String()
        if cmd == 'NONE':
            msg.data = 'NONE'
        else:
            msg.data = f'{cmd}:{wall_x:.4f}:{wall_y:.4f}'
        self.cmd_pub.publish(msg)

    # ── Anúncio único por parede ──────────────────────────────────────────────

    def _announce(self, color: str, wall_x: float, wall_y: float):
        info = self.COLOR_INFO[color]
        emoji, acao, cmd = info[1], info[0], info[3]
        self.get_logger().info(
            f'\n'
            f'  ╔══════════════════════════════════════╗\n'
            f'  ║  {emoji}  COR DETECTADA: {color:<6}             ║\n'
            f'  ║  Ação     : {acao:<26}║\n'
            f'  ║  Parede   : ({wall_x:6.2f}, {wall_y:6.2f})            ║\n'
            f'  ╚══════════════════════════════════════╝'
        )
        self._publish_marker(color, wall_x, wall_y)

    # ── Marcadores RViz ───────────────────────────────────────────────────────

    def _publish_marker(self, color: str, x: float, y: float):
        r, g, b, a = self.COLOR_INFO[color][2]
        zero = RosTime(sec=0, nanosec=0)
        ma   = MarkerArray()

        # Esfera colorida na posição do robô
        sphere = Marker()
        sphere.header.frame_id    = 'map'
        sphere.header.stamp       = zero
        sphere.ns                 = 'color_detections'
        sphere.id                 = self._marker_id
        sphere.type               = Marker.SPHERE
        sphere.action             = Marker.ADD
        sphere.pose.position.x    = x
        sphere.pose.position.y    = y
        sphere.pose.position.z    = 0.50
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.28
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = r, g, b, a
        sphere.lifetime.sec = 0
        ma.markers.append(sphere)

        self.marker_pub.publish(ma)
        self._marker_id += 3


# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
