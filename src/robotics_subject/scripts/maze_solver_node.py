#!/usr/bin/env python3
"""
maze_solver_node.py
-------------------
Autonomous maze solver — Regra da Mão Direita com odometria.

Rotação relativa (anti-drift)
-----------------------------
  Problema anterior: yaw absoluto como alvo → erro acumula a cada manobra
  por drift da odometria.

  Solução: mede QUANTO já girou desde o início da manobra, não onde quer
  chegar no mapa.

    yaw_inicio = snapshot ao chamar _start_rotation()
    ja_girado  = angle_diff(yaw_atual, yaw_inicio)
    erro_p     = |delta_alvo| - |ja_girado|   → vai a zero ao completar

  Controlador P usa esse erro relativo → desacelera antes de completar e
  não acumula drift entre manobras.

Manobras
--------
  TURN_LEFT     gira +90° (esquerda)
  U_TURN        -90° → avança até parede → -90° → avança até parede direita
  CORNER_RIGHT  -90° → avança até parede → -90° → avança até parede direita

Tópicos
-------
  Sub : /jetauto/lidar/scan        sensor_msgs/LaserScan
  Sub : /odometry/filtered         nav_msgs/Odometry
  Pub : /jetauto/cmd_vel           geometry_msgs/Twist
  Pub : /maze_status               std_msgs/String

Parâmetros
----------
  linear_speed       0.18  [m/s]
  rot_kp             2.00  ganho P do controlador de rotação
  rot_omega_max      1.00  [rad/s]  velocidade máxima de giro
  rot_omega_min      0.12  [rad/s]  velocidade mínima de giro (anti-stall)
  angle_tolerance    3.0   [°]      margem para considerar rotação concluída
  wall_distance      0.50  [m]      distância alvo da parede direita
  front_safe_dist    0.70  [m]      limiar base de obstáculo frontal
  brake_margin       0.25  [m]      margem extra de frenagem por inércia
  side_safe_dist     0.30  [m]      limiar obstáculo lateral (U-turn)
  open_area_thresh   2.50  [m]      todos lados > isso → saída
  startup_wall_dist  3.00  [m]      avança no startup até frente < este valor
  scan_angle_front   5     [°]      semi-cone "frente"
  scan_angle_side    10    [°]      semi-cone "laterais"
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String


# ─────────────────────────────────────────────────────────────────────────────
def _yaw_from_odom(odom: Odometry) -> float:
    q = odom.pose.pose.orientation
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _angle_diff(a: float, b: float) -> float:
    """a - b normalizado em [-π, π]."""
    d = a - b
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


def _min_range(ranges, start_idx: int, end_idx: int, max_range: float) -> float:
    n = len(ranges)
    si, ei = start_idx % n, end_idx % n
    indices = list(range(si, ei + 1)) if si <= ei else (
        list(range(si, n)) + list(range(0, ei + 1))
    )
    values = [ranges[i] for i in indices
              if not math.isnan(ranges[i]) and not math.isinf(ranges[i])]
    return min(values) if values else max_range


# ─────────────────────────────────────────────────────────────────────────────
class MazeSolverNode(Node):

    S_STARTUP = "STARTUP"
    S_FOLLOW  = "FOLLOW_WALL"
    S_TURN_L  = "TURN_LEFT"
    S_UTURN   = "U_TURN"
    S_CORNER  = "CORNER_RIGHT"
    S_SOLVED  = "SOLVED"

    # Sub-fases do U_TURN
    _UT_TURN1 = "ut_turn1"  # gira -90° (direita)
    _UT_FWD   = "ut_fwd"    # avança até encontrar parede frontal
    _UT_TURN2 = "ut_turn2"  # gira -90° (direita) novamente
    _UT_ALIGN = "ut_align"  # avança até parede direita reaparecer

    # Sub-fases do CORNER_RIGHT (idênticas, mesmos nomes por clareza)
    _CR_TURN1 = "cr_turn1"
    _CR_FWD   = "cr_fwd"
    _CR_TURN2 = "cr_turn2"
    _CR_ALIGN = "cr_align"

    def __init__(self):
        super().__init__('maze_solver')

        self.declare_parameter('linear_speed',      0.18)
        self.declare_parameter('rot_kp',            2.00)
        self.declare_parameter('rot_omega_max',     1.00)
        self.declare_parameter('rot_omega_min',     0.12)
        self.declare_parameter('angle_tolerance',   3.0)
        self.declare_parameter('wall_distance',     0.50)
        self.declare_parameter('front_safe_dist',   0.70)
        self.declare_parameter('brake_margin',      0.25)
        self.declare_parameter('side_safe_dist',    0.30)
        self.declare_parameter('open_area_thresh',  2.50)
        self.declare_parameter('startup_wall_dist', 3.00)
        self.declare_parameter('scan_angle_front',  5)
        self.declare_parameter('scan_angle_side',   10)
        self._load_params()

        self.cmd_pub    = self.create_publisher(Twist,  '/jetauto/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/maze_status',     10)
        self.create_subscription(LaserScan, '/jetauto/lidar/scan',
                                 self._scan_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered',
                                 self._odom_cb, 10)

        self.state           = self.S_STARTUP
        self.latest_scan     = None
        self.current_yaw     = None

        # Rotação relativa
        self._rot_start_yaw  = None   # yaw no momento em que a rotação começou
        self._rot_target_deg = None   # quantos graus girar (relativo, com sinal)

        self._uturn_phase    = self._UT_TURN1
        self._corner_phase   = self._CR_TURN1

        self.create_timer(0.10, self._loop)
        self.get_logger().info(
            f'MazeSolver iniciado — rotação relativa '
            f'(Kp={self.rot_kp}, ω_max={self.rot_omega_max:.2f}, '
            f'ω_min={self.rot_omega_min:.2f}, brake={self.brake_margin:.2f}m)')

    # ── parâmetros ─────────────────────────────────────────────────────────────
    def _load_params(self):
        self.v_lin          = self.get_parameter('linear_speed').value
        self.rot_kp         = self.get_parameter('rot_kp').value
        self.rot_omega_max  = self.get_parameter('rot_omega_max').value
        self.rot_omega_min  = self.get_parameter('rot_omega_min').value
        self.angle_tol      = self.get_parameter('angle_tolerance').value   # graus
        self.wall_dist      = self.get_parameter('wall_distance').value
        self.front_safe     = self.get_parameter('front_safe_dist').value
        self.brake_margin   = self.get_parameter('brake_margin').value
        self.side_safe      = self.get_parameter('side_safe_dist').value
        self.open_thresh    = self.get_parameter('open_area_thresh').value
        self.startup_wall_d = self.get_parameter('startup_wall_dist').value
        self.a_front        = self.get_parameter('scan_angle_front').value
        self.a_side         = self.get_parameter('scan_angle_side').value

    # ── callbacks ──────────────────────────────────────────────────────────────
    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def _odom_cb(self, msg: Odometry):
        self.current_yaw = _yaw_from_odom(msg)

    # ── limiar frontal dinâmico ────────────────────────────────────────────────
    @property
    def _front_threshold(self) -> float:
        moving = self.state in (self.S_FOLLOW, self.S_STARTUP, self.S_CORNER, self.S_UTURN)
        return self.front_safe + (self.brake_margin if moving else 0.0)

    # ── setores LaserScan ──────────────────────────────────────────────────────
    def _sectors(self, scan: LaserScan):
        n      = len(scan.ranges)
        step   = scan.angle_increment
        offset = round(-scan.angle_min / step)
        R      = scan.range_max

        def idx(deg):
            return int(offset + round(math.radians(deg) / step)) % n

        front = _min_range(scan.ranges, idx(-self.a_front),     idx(+self.a_front),     R)
        right = _min_range(scan.ranges, idx(-90 - self.a_side), idx(-90 + self.a_side), R)
        left  = _min_range(scan.ranges, idx(+90 - self.a_side), idx(+90 + self.a_side), R)
        return front, right, left

    # ── velocidade ────────────────────────────────────────────────────────────
    def _vel(self, lin: float, ang: float):
        msg = Twist()
        msg.linear.x  = float(lin)
        msg.angular.z = float(ang)
        self.cmd_pub.publish(msg)

    def _stop(self):
        self._vel(0.0, 0.0)

    # ── Controle de rotação relativa ───────────────────────────────────────────
    def _start_rotation(self, delta_deg: float):
        """
        Inicia uma rotação relativa.
        delta_deg > 0 → esquerda (CCW)
        delta_deg < 0 → direita  (CW)
        Salva o yaw atual como referência; o controlador mede quanto já girou.
        """
        self._rot_start_yaw  = self.current_yaw
        self._rot_target_deg = delta_deg

    def _rot_already_deg(self) -> float:
        """Quantos graus foram girados desde _start_rotation (com sinal)."""
        if self._rot_start_yaw is None:
            return 0.0
        return math.degrees(_angle_diff(self.current_yaw, self._rot_start_yaw))

    def _rotation_done(self) -> bool:
        """True quando o ângulo girado atingiu o alvo relativo."""
        if self._rot_start_yaw is None or self._rot_target_deg is None:
            return False
        already   = abs(self._rot_already_deg())
        remaining = abs(self._rot_target_deg) - already
        return remaining < self.angle_tol

    def _reset_rotation(self):
        self._rot_start_yaw  = None
        self._rot_target_deg = None

    def _p_omega(self, sign: float) -> float:
        """
        Controlador P com erro relativo.
          erro    = |delta_alvo| - |já_girado|
          omega   = clamp(Kp × erro_rad, ω_min, ω_max) × sign
        Desacelera naturalmente ao se aproximar do alvo.
        """
        if self._rot_start_yaw is None or self._rot_target_deg is None:
            return sign * self.rot_omega_min
        already   = abs(self._rot_already_deg())
        remaining = max(0.0, abs(self._rot_target_deg) - already)
        omega     = self.rot_kp * math.radians(remaining)
        omega     = max(self.rot_omega_min, min(self.rot_omega_max, omega))
        return sign * omega

    # ── loop principal (10 Hz) ─────────────────────────────────────────────────
    def _loop(self):
        if self.latest_scan is None or self.current_yaw is None:
            return
        if self.state == self.S_SOLVED:
            self._stop()
            return

        front, right, left = self._sectors(self.latest_scan)

        self.get_logger().debug(
            f'[{self.state}] F={front:.2f}(thr={self._front_threshold:.2f}) '
            f'R={right:.2f} L={left:.2f} yaw={math.degrees(self.current_yaw):.1f}°')

        # ══════════════════════════════════════════════════════════════════════
        # FSM
        # ══════════════════════════════════════════════════════════════════════

        # ── STARTUP ────────────────────────────────────────────────────────────
        if self.state == self.S_STARTUP:
            if front > self.startup_wall_d:
                self._vel(self.v_lin, 0.0)
            else:
                self.state = self.S_FOLLOW
                self.get_logger().info(
                    f'✅ Parede detectada a {front:.2f}m — wall-following iniciado.')
            self._pub_status(front, right, left)
            return

        # ── saída do labirinto ────────────────────────────────────────────────
        if (front > self.open_thresh and
                right > self.open_thresh and
                left  > self.open_thresh):
            self.state = self.S_SOLVED
            self._stop()
            self.get_logger().info('🎉 Labirinto resolvido!')
            s = String(); s.data = 'SOLVED'
            self.status_pub.publish(s)
            return

        # ── TURN_LEFT: gira +90° ──────────────────────────────────────────────
        if self.state == self.S_TURN_L:
            if self._rot_start_yaw is None:
                self._start_rotation(+90.0)
                self.get_logger().info(
                    f'↰ +90° (inicio={math.degrees(self._rot_start_yaw):.1f}°)')
            if self._rotation_done():
                self._reset_rotation()
                self.state = self.S_FOLLOW
                self._vel(self.v_lin, 0.0)
                self.get_logger().info('↰ +90° concluído.')
            else:
                self._vel(0.0, self._p_omega(+1.0))
            self._pub_status(front, right, left)
            return

        # ── U_TURN: -90° → avança → -90° → avança até parede direita ─────────
        if self.state == self.S_UTURN:

            if self._uturn_phase == self._UT_TURN1:
                if self._rot_start_yaw is None:
                    self._start_rotation(-90.0)
                    self.get_logger().info('↩️  U-turn fase 1: -90°')
                if self._rotation_done():
                    self._reset_rotation()
                    self._uturn_phase = self._UT_FWD
                    self.get_logger().info('↩️  U-turn fase 1 concluída — avançando.')
                else:
                    self._vel(0.0, self._p_omega(-1.0))

            elif self._uturn_phase == self._UT_FWD:
                if front < self._front_threshold:
                    self._uturn_phase = self._UT_TURN2
                    self.get_logger().info('↩️  Parede encontrada — U-turn fase 3: -90°')
                else:
                    self._vel(self.v_lin, 0.0)

            elif self._uturn_phase == self._UT_TURN2:
                if self._rot_start_yaw is None:
                    self._start_rotation(-90.0)
                    self.get_logger().info('↩️  U-turn fase 3: -90°')
                if self._rotation_done():
                    self._reset_rotation()
                    self._uturn_phase = self._UT_ALIGN
                    self.get_logger().info('↩️  U-turn fase 3 concluída — alinhando.')
                else:
                    self._vel(0.0, self._p_omega(-1.0))

            elif self._uturn_phase == self._UT_ALIGN:
                if right <= self.wall_dist + 1.0:
                    self._uturn_phase = self._UT_TURN1  # reseta para próxima vez
                    self.state        = self.S_FOLLOW
                    self._vel(self.v_lin, 0.0)
                    self.get_logger().info('↩️  U-turn concluído — parede direita encontrada.')
                else:
                    self._vel(self.v_lin, 0.0)

            self._pub_status(front, right, left)
            return

        # ── CORNER_RIGHT: -90° → avança → -90° → avança até parede direita ───
        if self.state == self.S_CORNER:

            if self._corner_phase == self._CR_TURN1:
                if self._rot_start_yaw is None:
                    self._start_rotation(-90.0)
                    self.get_logger().info('↪️  Corner fase 1: -90°')
                if self._rotation_done():
                    self._reset_rotation()
                    self._corner_phase = self._CR_FWD
                    self.get_logger().info('↪️  Corner fase 1 concluída — avançando.')
                else:
                    self._vel(0.0, self._p_omega(-1.0))

            elif self._corner_phase == self._CR_FWD:
                if front < self._front_threshold:
                    self._corner_phase = self._CR_TURN2
                    self.get_logger().info('↪️  Parede encontrada — corner fase 3: -90°')
                else:
                    self._vel(self.v_lin, 0.0)

            elif self._corner_phase == self._CR_TURN2:
                if self._rot_start_yaw is None:
                    self._start_rotation(-90.0)
                    self.get_logger().info('↪️  Corner fase 3: -90°')
                if self._rotation_done():
                    self._reset_rotation()
                    self._corner_phase = self._CR_ALIGN
                    self.get_logger().info('↪️  Corner fase 3 concluída — alinhando.')
                else:
                    self._vel(0.0, self._p_omega(-1.0))

            elif self._corner_phase == self._CR_ALIGN:
                if right <= self.wall_dist + 1.0:
                    self._corner_phase = self._CR_TURN1  # reseta para próxima vez
                    self.state         = self.S_FOLLOW
                    self._vel(self.v_lin, 0.0)
                    self.get_logger().info('↪️  Corner concluído — parede direita encontrada.')
                else:
                    self._vel(self.v_lin, 0.0)

            self._pub_status(front, right, left)
            return

        # ── Detecção de U-turn (frente E esquerda bloqueadas) ─────────────────
        if front < self._front_threshold and left < self.side_safe:
            self.state        = self.S_UTURN
            self._uturn_phase = self._UT_TURN1
            self._reset_rotation()
            self._pub_status(front, right, left)
            return

        # ── Só frente bloqueada → TURN_LEFT ───────────────────────────────────
        if front < self._front_threshold:
            self.state = self.S_TURN_L
            self._reset_rotation()
            self._pub_status(front, right, left)
            return

        # ── Parede direita sumiu → CORNER_RIGHT ──────────────────────────────
        if right > self.wall_dist + 1.0:
            if self.state != self.S_CORNER:
                self.state         = self.S_CORNER
                self._corner_phase = self._CR_TURN1
                self._reset_rotation()
                self.get_logger().info('↪️  Parede direita sumiu — corner direita.')
            self._vel(self.v_lin, 0.0)

        # ── Perto demais da parede direita → ajuste suave ─────────────────────
        elif right < self.wall_dist - 0.10:
            self.state = self.S_FOLLOW
            self._vel(self.v_lin * 0.8, self.rot_omega_min * 0.3)

        # ── Distância ideal → segue em frente ────────────────────────────────
        else:
            self.state = self.S_FOLLOW
            self._vel(self.v_lin, 0.0)

        self._pub_status(front, right, left)

    # ── status ────────────────────────────────────────────────────────────────
    def _pub_status(self, f, r, l):
        yaw_str = f'{math.degrees(self.current_yaw):.1f}°' if self.current_yaw is not None else '?'
        if self._rot_start_yaw is not None and self._rot_target_deg is not None:
            already   = abs(self._rot_already_deg())
            remaining = abs(self._rot_target_deg) - already
            rot_str   = f'rot={already:.1f}°/tgt={self._rot_target_deg:.0f}° rem={remaining:.1f}°'
        else:
            rot_str = '-'
        s = String()
        s.data = (f'state={self.state} F={f:.2f}/thr={self._front_threshold:.2f} '
                  f'R={r:.2f} L={l:.2f} yaw={yaw_str} {rot_str}')
        self.status_pub.publish(s)


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = MazeSolverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()