#!/usr/bin/env python3
"""
maze_nav2_node.py  —  robotics_subject / JetAuto

Pipeline (a cada replan_interval segundos):
  1. Detecta fronteiras no mapa SLAM
  2. Seleciona melhor fronteira por score (distância, tamanho, revisita, exclusão inicial)
  3. Calcula A* com penalidade de proximidade de parede
  4. Simplifica com RDP
  5. Escolhe o primeiro ponto a >= waypoint_dist m do robô com LoS livre
     (sem parede entre o robô e o ponto no grid SLAM)
     Se o ponto escolhido tiver parede no LoS, recua até achar LoS livre
  6. Envia ao Nav2 via NavigateToPose (fire-and-forget, replaneja por timer)
"""

import heapq
import math
from typing import List, Optional, Tuple

import numpy as np
from scipy.ndimage import distance_transform_edt, binary_dilation

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
from builtin_interfaces.msg import Time as RosTime
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose


# ══════════════════════════════════════════════════════════════════════════════
# Helpers
# ══════════════════════════════════════════════════════════════════════════════

def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)

def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return 0.0, 0.0, sy, cy

def angle_diff(a: float, b: float) -> float:
    d = a - b
    while d >  math.pi: d -= 2.0 * math.pi
    while d < -math.pi: d += 2.0 * math.pi
    return d

def sector_min(ranges, angle_min, angle_inc, center_deg, half_deg, max_r) -> float:
    lo = math.radians(center_deg - half_deg)
    hi = math.radians(center_deg + half_deg)
    vals = [r for i, r in enumerate(ranges)
            if lo <= angle_min + i * angle_inc <= hi and math.isfinite(r) and r > 0.0]
    return min(vals) if vals else max_r

def range_at(ranges, angle_min, angle_inc, deg, window_deg=8.0, max_r=10.0) -> float:
    rad = math.radians(deg)
    hw  = math.radians(window_deg)
    vals = [r for i, r in enumerate(ranges)
            if abs(angle_min + i * angle_inc - rad) <= hw and math.isfinite(r) and r > 0.0]
    return min(vals) if vals else max_r


# ══════════════════════════════════════════════════════════════════════════════
# Grid algorithms
# ══════════════════════════════════════════════════════════════════════════════

_DIRS8: List[Tuple[int, int, float]] = [
    ( 1,  0, 1.000), (-1,  0, 1.000), ( 0,  1, 1.000), ( 0, -1, 1.000),
    ( 1,  1, 1.414), ( 1, -1, 1.414), (-1,  1, 1.414), (-1, -1, 1.414),
]


def wall_penalty_map(grid: np.ndarray, lethal_thresh: int,
                     penalty_max: float, radius_cells: float) -> np.ndarray:
    """Penalidade inversamente proporcional à distância da parede mais próxima."""
    obstacle = (grid > lethal_thresh) | (grid < 0)
    dist = distance_transform_edt(~obstacle).astype(np.float32)
    return np.clip(1.0 - dist / radius_cells, 0.0, 1.0) * penalty_max


def inflate_grid(grid: np.ndarray, lethal_thresh: int,
                 radius_cells: int) -> np.ndarray:
    """
    Dilata todas as células letais (>lethal_thresh) por radius_cells células.
    Retorna um novo grid com o mesmo dtype onde as células dilatadas valem 100.
    Células desconhecidas (< 0) não são dilatadas mas são preservadas.

    radius_cells=2 expande cada pixel de parede num bloco de ~4px de raio,
    impedindo que o A* planeje rotas que passem a menos de 2 células de uma parede.
    """
    if radius_cells <= 0:
        return grid.copy()
    lethal_mask = grid > lethal_thresh
    struct = np.ones((2 * radius_cells + 1, 2 * radius_cells + 1), dtype=bool)
    dilated = binary_dilation(lethal_mask, structure=struct)
    result = grid.copy()
    # Marca como letal (100) apenas células livres/conhecidas que foram dilatadas;
    # não sobrescreve células desconhecidas (-1) para não bloquear exploração.
    inflate_mask = dilated & ~lethal_mask & (grid >= 0)
    result[inflate_mask] = 100
    return result

def los_clear(grid: np.ndarray, c0: int, r0: int,
              c1: int, r1: int, lethal_thresh: int) -> bool:
    """Bresenham: True se a linha c0,r0 → c1,r1 não cruza célula letal."""
    rows, cols = grid.shape
    steps = max(abs(c1 - c0), abs(r1 - r0))
    if steps == 0:
        return True
    for i in range(steps + 1):
        t = i / steps
        c = int(round(c0 + t * (c1 - c0)))
        r = int(round(r0 + t * (r1 - r0)))
        if not (0 <= c < cols and 0 <= r < rows):
            return False
        if int(grid[r, c]) > lethal_thresh:
            return False
    return True


def reachable(grid: np.ndarray, start: Tuple[int, int],
              goal: Tuple[int, int], lethal_thresh: int) -> bool:
    """BFS 8-viz: verifica conectividade antes de rodar A*."""
    rows, cols = grid.shape

    def ok(c, r):
        return 0 <= c < cols and 0 <= r < rows and int(grid[r, c]) <= lethal_thresh

    if not ok(*start) or not ok(*goal):
        return False
    if start == goal:
        return True
    visited = {start}
    queue = [start]
    while queue:
        c, r = queue.pop()
        for dc, dr in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
            nb = (c + dc, r + dr)
            if nb not in visited and ok(*nb):
                if nb == goal:
                    return True
                visited.add(nb)
                queue.append(nb)
    return False


def astar(grid: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int],
          free_cost: float, unknown_cost: float, lethal_thresh: int,
          penalty_fn) -> Optional[List[Tuple[int, int]]]:
    """A* 8-conectado com penalidade de parede."""
    rows, cols = grid.shape
    sc, sr = start
    gc = max(0, min(cols - 1, goal[0]))
    gr = max(0, min(rows - 1, goal[1]))

    if not (0 <= sc < cols and 0 <= sr < rows):
        return None

    def cost(c, r):
        if not (0 <= c < cols and 0 <= r < rows):
            return float('inf')
        v = int(grid[r, c])
        if v > lethal_thresh:
            return float('inf')
        base = unknown_cost if v < 0 else free_cost
        if v >= 0:
            base = min(base + penalty_fn(c, r), unknown_cost - 1e-6)
        return base

    def h(c, r):
        return math.hypot(c - gc, r - gr)

    heap = [(h(sc, sr), sc, sr)]
    came_from: dict = {}
    g: dict = {(sc, sr): 0.0}

    while heap:
        f, c, r = heapq.heappop(heap)
        if (c, r) == (gc, gr):
            path, cur = [], (gc, gr)
            while cur in came_from:
                path.append(cur)
                cur = came_from[cur]
            path.append((sc, sr))
            path.reverse()
            return path
        if f > g.get((c, r), float('inf')) + h(c, r) + 1e-9:
            continue
        cg = g[(c, r)]
        for dc, dr, step in _DIRS8:
            nc, nr = c + dc, r + dr
            cc = cost(nc, nr)
            if cc == float('inf'):
                continue
            ng = cg + step * cc
            key = (nc, nr)
            if ng < g.get(key, float('inf')):
                g[key] = ng
                came_from[key] = (c, r)
                heapq.heappush(heap, (ng + h(nc, nr), nc, nr))
    return None


def rdp_simplify(path: List[Tuple[int, int]], epsilon: float) -> List[Tuple[int, int]]:
    """Ramer-Douglas-Peucker: mantém viradas, remove pontos colineares."""
    if len(path) <= 2:
        return path

    def pdist(p, a, b):
        ax, ay = a; bx, by = b; px, py = p
        dx, dy = bx - ax, by - ay
        if dx == dy == 0:
            return math.hypot(px - ax, py - ay)
        t = max(0.0, min(1.0, ((px - ax)*dx + (py - ay)*dy) / (dx*dx + dy*dy)))
        return math.hypot(px - ax - t*dx, py - ay - t*dy)

    def _rdp(pts, eps):
        if len(pts) <= 2:
            return pts
        dmax, idx = 0.0, 0
        for i in range(1, len(pts) - 1):
            d = pdist(pts[i], pts[0], pts[-1])
            if d > dmax:
                dmax, idx = d, i
        if dmax > eps:
            return _rdp(pts[:idx + 1], eps)[:-1] + _rdp(pts[idx:], eps)
        return [pts[0], pts[-1]]

    return _rdp(path, epsilon)


# ══════════════════════════════════════════════════════════════════════════════
# Node
# ══════════════════════════════════════════════════════════════════════════════

class MazeNav2Node(Node):

    S_INIT    = "INIT"
    S_STARTUP = "STARTUP"
    S_NAV     = "NAVIGATING"
    S_SOLVED  = "SOLVED"

    def __init__(self):
        super().__init__('maze_nav2')

        self.declare_parameter('startup_wall_dist',     2.00)
        self.declare_parameter('startup_lin_speed',     0.36)
        self.declare_parameter('open_area_thresh',      2.50)
        self.declare_parameter('waypoint_dist',         0.60)
        self.declare_parameter('prefer_right',          True)
        self.declare_parameter('min_frontier_size_m',   0.30)
        self.declare_parameter('visited_radius',        0.25)
        self.declare_parameter('backtrack_penalty',     2.50)
        self.declare_parameter('forward_bonus',         1.20)
        self.declare_parameter('astar_unknown_cost',    3.0)
        self.declare_parameter('astar_lethal_thresh',   50)
        self.declare_parameter('show_frontier_markers', True)
        self.declare_parameter('wall_penalty_max',      12.0)
        self.declare_parameter('wall_penalty_radius_m', 0.50)
        self.declare_parameter('wall_inflate_cells',    5)     # raio de dilatação do A*
        self.declare_parameter('rdp_epsilon_m',         0.15)
        self.declare_parameter('replan_interval_sec',   0.5)
        self.declare_parameter('start_excl_radius',     2.00)
        self.declare_parameter('start_excl_cost',       50.0)

        p = self.get_parameter
        self.startup_wall_d    = p('startup_wall_dist').value
        self.startup_lin_spd   = p('startup_lin_speed').value
        self.open_thresh       = p('open_area_thresh').value
        self.waypoint_dist     = p('waypoint_dist').value
        self.prefer_right      = p('prefer_right').value
        self.min_frontier_m    = p('min_frontier_size_m').value
        self.visited_radius    = p('visited_radius').value
        self.backtrack_penalty = p('backtrack_penalty').value
        self.forward_bonus     = p('forward_bonus').value
        self.unknown_cost      = p('astar_unknown_cost').value
        self.lethal_thresh     = int(p('astar_lethal_thresh').value)
        self.show_frontier_mrk = p('show_frontier_markers').value
        self.wall_pen_max      = p('wall_penalty_max').value
        self.wall_pen_radius   = p('wall_penalty_radius_m').value
        self.wall_inflate_cells = int(p('wall_inflate_cells').value)
        self.rdp_epsilon_m     = p('rdp_epsilon_m').value
        self.replan_interval   = p('replan_interval_sec').value
        self.start_excl_radius = p('start_excl_radius').value
        self.start_excl_cost   = p('start_excl_cost').value

        self.state           = self.S_INIT
        self.latest_scan     = None
        self.current_pose    = None
        self._slam_map       = None
        self._goal_handle    = None
        self._current_wp:  Optional[Tuple[float, float]] = None
        self._last_replan: Optional[object] = None
        self._start_pos:   Optional[Tuple[float, float]] = None
        self._visited_goals: List[Tuple[float, float]] = []
        self._wp_count     = 0
        self._wp_trail:    List[Tuple[float, float]] = []

        # Zonas de restrição por cor: lista de (x, y, yaw, direction)
        # Cada entrada bloqueia fronteiras no lado oposto dentro de COLOR_ZONE_RADIUS.
        self._color_goals: List[Tuple[float, float, float, str]] = []
        self._COLOR_DEDUPE_DIST  = 2.5   # m — não registrar a mesma zona duas vezes
        self._COLOR_ZONE_RADIUS  = 6.0   # m — raio de influência da restrição

        # Células do grid SLAM marcadas como letais virtualmente.
        # Aplicadas por cima do mapa real em _grid() — o A* e o LoS check
        # as tratam exatamente como paredes reais, sem nenhuma lógica extra.
        self._virtual_wall_cells: List[Tuple[int, int]] = []

        self.cmd_pub        = self.create_publisher(Twist,       '/jetauto/cmd_vel',    10)
        self.status_pub     = self.create_publisher(String,      '/maze_status',        10)
        self.marker_pub     = self.create_publisher(MarkerArray, '/maze_waypoints',     10)
        self.frontier_pub   = self.create_publisher(MarkerArray, '/maze_frontiers',     10)
        self.color_zone_pub = self.create_publisher(MarkerArray, '/maze_color_zones',   10)
        self.path_pub       = self.create_publisher(MarkerArray, '/maze_astar_path',    10)

        _tl = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(LaserScan,     '/jetauto/lidar/scan', self._scan_cb, 10)
        self.create_subscription(Odometry,      '/odometry/filtered',  self._odom_cb, 10)
        self.create_subscription(OccupancyGrid, '/map',                self._map_cb,  _tl)
        self.create_subscription(String,        '/color_command',      self._color_cmd_cb, 10)

        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.create_timer(0.20, self._loop)
        self.get_logger().info('MazeNav2 iniciado.')

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _scan_cb(self, msg): self.latest_scan = msg
    def _map_cb(self, msg):  self._slam_map   = msg

    def _odom_cb(self, msg):
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quat(msg.pose.pose.orientation),
        )

    def _color_cmd_cb(self, msg: String):
        """
        Recebe comando do color_detector_node.
        Formato: "LEFT:wall_x:wall_y" | "RIGHT:wall_x:wall_y" | "NONE"

        (wall_x, wall_y) é a posição estimada da PAREDE no mapa (via lidar +
        centroide da cor na imagem), calculada pelo color_detector_node.
        O yaw do robô no momento é usado apenas para definir o eixo lateral
        (esquerda/direita) relativo ao corredor.

        Deduplicação leve: ignora se já existe uma zona registrada desta cor
        a menos de _COLOR_DEDUPE_DIST metros da mesma posição de parede.
        """
        if self.state != self.S_NAV or self.current_pose is None:
            return

        raw = msg.data
        if raw == 'NONE':
            return

        # Parseia "DIRECTION:wall_x:wall_y"
        parts = raw.split(':')
        if len(parts) != 3:
            self.get_logger().warn(f'[COR] Formato inválido: "{raw}"')
            return
        direction = parts[0]
        try:
            wall_x = float(parts[1])
            wall_y = float(parts[2])
        except ValueError:
            self.get_logger().warn(f'[COR] Coordenadas inválidas: "{raw}"')
            return

        # ryaw do robô define o eixo lateral (qual lado é esquerda/direita)
        _, _, ryaw = self.current_pose

        # Deduplicação pela posição da parede
        for cx, cy, _, cdir in self._color_goals:
            if math.hypot(wall_x - cx, wall_y - cy) < self._COLOR_DEDUPE_DIST and cdir == direction:
                return

        self._color_goals.append((wall_x, wall_y, ryaw, direction))
        lado_parede = 'ESQUERDA' if direction == 'LEFT' else 'DIREITA'
        lado_bloq   = 'direita'  if direction == 'LEFT' else 'esquerda'
        self.get_logger().info(
            f'[COR] Parede {lado_parede} em ({wall_x:.2f},{wall_y:.2f}) '
            f'yaw_robô={math.degrees(ryaw):.1f}° → '
            f'bloqueando waypoints à {lado_bloq} num raio de {self._COLOR_ZONE_RADIUS:.1f}m'
        )
        self._publish_color_zone_markers()

    def _is_wp_forbidden(self, wx: float, wy: float) -> bool:
        """
        Retorna True se o waypoint (wx, wy) está numa zona de restrição de cor.

        (dx, dy) é a posição do robô quando recebeu o comando de cor.
        dyaw é o heading nesse momento — define o eixo esquerda/direita.

        Um waypoint é proibido se:
          1. Está dentro de COLOR_ZONE_RADIUS ao redor de (dx, dy).
          2. Não há parede entre (dx, dy) e (wx, wy) no grid SLAM — evita que
             a zona de restrição "atravesse" a parede para o corredor oposto.
          3. Está no lado errado (componente lateral em relação a dyaw):
               RED  (LEFT)  → bloqueia pontos à DIREITA  (lat < 0)
               GREEN (RIGHT) → bloqueia pontos à ESQUERDA (lat > 0)
        """
        grid = self._grid()

        for dx, dy, dyaw, direction in self._color_goals:
            if math.hypot(wx - dx, wy - dy) > self._COLOR_ZONE_RADIUS:
                continue

            # ── Teste de parede (LoS) ────────────────────────────────────────
            # Se o grid está disponível, só aplica a restrição se não há parede
            # entre o ponto de registro e o waypoint. Isso impede que a zona
            # de restrição atravesse paredes para corredores do lado oposto.
            if grid is not None:
                info = self._slam_map.info
                dc, dr = self._w2g(info, dx, dy)
                wc, wr = self._w2g(info, wx, wy)
                if not los_clear(grid, dc, dr, wc, wr, self.lethal_thresh):
                    continue   # parede no caminho — restrição não se aplica

            # Vetor do ponto de registro até o waypoint
            vx, vy = wx - dx, wy - dy

            # Componente lateral (positivo = esquerda do robô, negativo = direita)
            lat = -vx * math.sin(dyaw) + vy * math.cos(dyaw)

            if direction == 'LEFT'  and lat < 0.0:   # RED:   bloqueia lado direito
                return True
            if direction == 'RIGHT' and lat > 0.0:   # GREEN: bloqueia lado esquerdo
                return True

        return False

    # ── grid helpers ──────────────────────────────────────────────────────────

    def _w2g(self, info, wx, wy) -> Tuple[int, int]:
        return (int((wx - info.origin.position.x) / info.resolution),
                int((wy - info.origin.position.y) / info.resolution))

    def _g2w(self, info, c, r) -> Tuple[float, float]:
        return (info.origin.position.x + (c + 0.5) * info.resolution,
                info.origin.position.y + (r + 0.5) * info.resolution)

    def _grid(self) -> Optional[np.ndarray]:
        if self._slam_map is None:
            return None
        i = self._slam_map.info
        grid = np.array(self._slam_map.data, dtype=np.int8).reshape((i.height, i.width))
        # Aplica paredes virtuais por cima do mapa real.
        # O valor 100 é o máximo da OccupancyGrid (célula completamente ocupada).
        for c, r in self._virtual_wall_cells:
            if 0 <= r < i.height and 0 <= c < i.width:
                grid[r, c] = 100
        return grid

    # ── A* planner ────────────────────────────────────────────────────────────

    def _plan_astar(self, goal_wx: float, goal_wy: float
                    ) -> Optional[List[Tuple[float, float]]]:
        if self.current_pose is None or self._slam_map is None:
            return None

        grid = self._grid()
        info = self._slam_map.info
        rx, ry, _ = self.current_pose

        # Grid inflado: usado exclusivamente pelo A* para manter distância das paredes.
        # O grid original (não inflado) continua sendo usado para LoS check e fronteiras.
        astar_grid = inflate_grid(grid, self.lethal_thresh, self.wall_inflate_cells)

        r_cells = max(1.0, self.wall_pen_radius / info.resolution)
        wpm = wall_penalty_map(astar_grid, self.lethal_thresh, self.wall_pen_max, r_cells)

        excl_active = self._start_pos is not None
        sc_excl = (0, 0)
        r_excl  = 0.0
        if excl_active:
            sc_excl = self._w2g(info, *self._start_pos)
            r_excl  = self.start_excl_radius / info.resolution

        def penalty(c, r):
            pen = float(wpm[r, c])
            if excl_active and r_excl > 0:
                d = math.hypot(c - sc_excl[0], r - sc_excl[1])
                if d < r_excl:
                    pen += self.start_excl_cost * (1.0 - d / r_excl)
            return pen

        start_g = self._w2g(info, rx, ry)
        goal_g  = self._w2g(info, goal_wx, goal_wy)

        if not reachable(astar_grid, start_g, goal_g, self.lethal_thresh):
            return None

        path_cells = astar(astar_grid, start_g, goal_g,
                           free_cost=1.0, unknown_cost=self.unknown_cost,
                           lethal_thresh=self.lethal_thresh, penalty_fn=penalty)
        if not path_cells:
            return None

        rdp_eps    = max(0.5, self.rdp_epsilon_m / info.resolution)
        simplified = rdp_simplify(path_cells, rdp_eps)

        # filtra waypoints letais ou encostados em parede; descarta path[0] (posição do robô)
        safe = []
        for c, r in simplified[1:]:
            if not (0 <= c < info.width and 0 <= r < info.height):
                continue
            if int(astar_grid[r, c]) > self.lethal_thresh:
                continue
            if any(int(astar_grid[r + dr, c + dc]) > self.lethal_thresh
                   for dc, dr in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]
                   if 0 <= c + dc < info.width and 0 <= r + dr < info.height):
                continue
            safe.append((c, r))

        if not safe:
            return None

        # Retorna (path_simplificado, path_completo_em_metros)
        full_path = [self._g2w(info, c, r) for c, r in path_cells]
        return [self._g2w(info, c, r) for c, r in safe], full_path

    # ── fronteiras ────────────────────────────────────────────────────────────

    def _find_frontiers(self) -> List[Tuple[float, float]]:
        if self._slam_map is None or self.current_pose is None:
            return []

        data = self._slam_map.data
        info = self._slam_map.info
        w, h = info.width, info.height
        res  = info.resolution
        rx, ry, ryaw = self.current_pose
        min_cells = max(3, int(self.min_frontier_m / res))

        frontier: set = set()
        for row in range(1, h - 1):
            for col in range(1, w - 1):
                if data[row * w + col] != 0:
                    continue
                n4 = [data[(row+dr)*w+(col+dc)] for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]]
                n8 = [data[(row+dr)*w+(col+dc)] for dr in [-1,0,1] for dc in [-1,0,1]
                      if not (dr == 0 and dc == 0)]
                if any(v == -1 for v in n4) and not any(v > 50 for v in n8):
                    frontier.add((col, row))

        if not frontier:
            return []

        visited: set = set()
        clusters: List[Tuple[float, float, int]] = []
        for seed in frontier:
            if seed in visited:
                continue
            cluster, queue = [], [seed]
            while queue:
                cell = queue.pop()
                if cell in visited:
                    continue
                visited.add(cell)
                if cell in frontier:
                    cluster.append(cell)
                    cc_, rc_ = cell
                    for nb in [(cc_+1,rc_),(cc_-1,rc_),(cc_,rc_+1),(cc_,rc_-1)]:
                        if nb not in visited:
                            queue.append(nb)
            if len(cluster) >= min_cells:
                cc = int(sum(c for c, _ in cluster) / len(cluster))
                cr = int(sum(r for _, r in cluster) / len(cluster))
                wx = info.origin.position.x + (cc + 0.5) * res
                wy = info.origin.position.y + (cr + 0.5) * res
                clusters.append((wx, wy, len(cluster)))

        if not clusters:
            return []

        def score(f):
            fx, fy, size = f
            dist    = math.hypot(fx - rx, fy - ry) + 0.01
            rel_ang = abs(angle_diff(math.atan2(fy - ry, fx - rx), ryaw))
            fwd_b   = self.forward_bonus * max(0.0, 1.0 - rel_ang / math.pi)
            size_b  = size * res * res * 0.4
            bt_pen  = sum(self.backtrack_penalty
                          * max(0.0, 1.0 - math.hypot(fx-vx, fy-vy) / self.visited_radius)
                          for vx, vy in self._visited_goals)
            excl_pen = 0.0
            if self._start_pos is not None:
                d = math.hypot(fx - self._start_pos[0], fy - self._start_pos[1])
                if d < self.start_excl_radius:
                    excl_pen = self.start_excl_cost * (1.0 - d / self.start_excl_radius)
            return dist - fwd_b - size_b + bt_pen + excl_pen

        clusters.sort(key=score)
        return [(wx, wy) for wx, wy, _ in clusters
                if not any(math.hypot(wx-vx, wy-vy) < self.visited_radius * 0.5
                           for vx, vy in self._visited_goals)]

    # ── próximo waypoint ─────────────────────────────────────────────────────

    def _plan_next_wp(self) -> Optional[Tuple[float, float]]:
        """
        Para a melhor fronteira com A* viável:
          1. Percorre o path RDP procurando o primeiro ponto a >= waypoint_dist
          2. Verifica LoS (Bresenham) entre o robô e esse ponto no grid SLAM
          3. Se LoS bloqueado por parede, recua no path até achar LoS livre
        """
        if self.current_pose is None or self._slam_map is None:
            return None

        rx, ry, _ = self.current_pose
        grid = self._grid()
        info = self._slam_map.info
        start_g = self._w2g(info, rx, ry)

        frontiers = self._find_frontiers()
        # Remove fronteiras que caem em zonas de restrição de cor
        frontiers = [f for f in frontiers if not self._is_wp_forbidden(*f)]
        if frontiers:
            self._publish_frontier_markers(frontiers[:8])

        for fx, fy in frontiers[:8]:
            result = self._plan_astar(fx, fy)
            if not result:
                continue
            path, full_path = result

            # converte para grid para o LoS check
            path_g = [self._w2g(info, wx, wy) for wx, wy in path]

            # encontra primeiro ponto a >= waypoint_dist
            chosen_idx = len(path) - 1
            for i, (wx, wy) in enumerate(path):
                if math.hypot(wx - rx, wy - ry) >= self.waypoint_dist:
                    chosen_idx = i
                    break

            # recua enquanto LoS estiver bloqueado por parede
            while chosen_idx > 0:
                pc, pr = path_g[chosen_idx]
                if los_clear(grid, start_g[0], start_g[1], pc, pr, self.lethal_thresh):
                    break
                chosen_idx -= 1

            wx, wy = path[chosen_idx]
            if self._is_wp_forbidden(wx, wy):
                continue   # waypoint intermediário cai em zona proibida por cor

            # Mapeia o waypoint escolhido para o índice correspondente no full_path
            full_chosen_idx = min(
                range(len(full_path)),
                key=lambda i: math.hypot(full_path[i][0] - wx, full_path[i][1] - wy)
            )

            self.get_logger().info(
                f'Fronteira ({fx:.2f},{fy:.2f}) → wp ({wx:.2f},{wy:.2f}) '
                f'dist={math.hypot(wx-rx, wy-ry):.2f}m')
            self._publish_path_marker(full_path, full_chosen_idx)
            return (wx, wy)

        # fallback scan
        self.get_logger().warn('Sem A* viável — fallback scan.')
        return self._scan_fallback()

    def _scan_fallback(self) -> Optional[Tuple[float, float]]:
        if self.latest_scan is None:
            return None
        scan         = self.latest_scan
        rx, ry, ryaw = self.current_pose
        am, ai, R    = scan.angle_min, scan.angle_increment, scan.range_max

        best_deg, best_val = 0.0, -1.0
        for deg in range(-150, 151, 5):
            v = sector_min(scan.ranges, am, ai, deg, 25, R)
            bonus = 0.04 if (self.prefer_right and deg < 0) else 0.0
            if v + bonus > best_val:
                best_val, best_deg = v + bonus, float(deg)

        angle_w = ryaw + math.radians(best_deg)
        d = max(0.35, min(self.waypoint_dist,
                          range_at(scan.ranges, am, ai, best_deg, 10.0, R) * 0.55))
        return (rx + d * math.cos(angle_w), ry + d * math.sin(angle_w))

    # ── Nav2 ─────────────────────────────────────────────────────────────────

    def _send_wp(self, wx: float, wy: float) -> bool:
        if not self._nav_client.wait_for_server(timeout_sec=0.1):
            return False
        self._cancel_goal()

        rx, ry, _ = self.current_pose
        qx, qy, qz, qw = yaw_to_quat(math.atan2(wy - ry, wx - rx))

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id    = 'map'
        goal.pose.header.stamp       = RosTime(sec=0, nanosec=0)
        goal.pose.pose.position.x    = wx
        goal.pose.pose.position.y    = wy
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self._current_wp  = (wx, wy)
        self._last_replan = self.get_clock().now()
        self._visited_goals.append((wx, wy))
        self._publish_wp_marker(wx, wy)

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_accepted_cb)
        self.get_logger().info(f'WP → ({wx:.2f},{wy:.2f})')
        return True

    def _goal_accepted_cb(self, future):
        handle = future.result()
        if handle and handle.accepted:
            self._goal_handle = handle

    def _cancel_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    # ── marcadores RViz ───────────────────────────────────────────────────────

    def _publish_color_zone_markers(self):
        """
        Publica no RViz (/maze_color_zones) dois grupos de marcadores:

        A) Para cada zona de cor (_color_goals):
           • Esfera no ponto de registro (cor = vermelha/verde)
           • Seta indicando o lado BLOQUEADO
           • Cilindro semitransparente mostrando o raio COLOR_ZONE_RADIUS

        B) Para as células de parede virtual (_virtual_wall_cells):
           • Cubo laranja em cada célula do grid
        """
        if self._slam_map is None:
            return

        zero = RosTime(sec=0, nanosec=0)
        ma   = MarkerArray()

        # ── Limpa marcadores anteriores ───────────────────────────────────────
        clr = Marker()
        clr.header.frame_id = 'map'; clr.header.stamp = zero
        clr.ns = 'cz_zones'; clr.id = 0; clr.action = Marker.DELETEALL
        ma.markers.append(clr)
        clr2 = Marker()
        clr2.header.frame_id = 'map'; clr2.header.stamp = zero
        clr2.ns = 'cz_vwall'; clr2.id = 0; clr2.action = Marker.DELETEALL
        ma.markers.append(clr2)

        # Paleta por direção
        # LEFT (vermelho) → zona proibida à direita → mostra em vermelho
        # RIGHT (verde)   → zona proibida à esquerda → mostra em verde
        PALETTE = {
            'LEFT':  (1.0, 0.15, 0.15),   # vermelho
            'RIGHT': (0.15, 1.0, 0.15),   # verde
        }

        mid = 0
        for idx, (dx, dy, dyaw, direction) in enumerate(self._color_goals):
            cr, cg, cb = PALETTE.get(direction, (1.0, 1.0, 0.0))

            # ── Esfera no ponto de registro ───────────────────────────────────
            sphere = Marker()
            sphere.header.frame_id = 'map'; sphere.header.stamp = zero
            sphere.ns = 'cz_zones'; sphere.id = mid; mid += 1
            sphere.type = Marker.SPHERE; sphere.action = Marker.ADD
            sphere.pose.position.x = dx
            sphere.pose.position.y = dy
            sphere.pose.position.z = 0.40
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.22
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = cr, cg, cb, 1.0
            sphere.lifetime.sec = 0
            ma.markers.append(sphere)

            # ── Rótulo de texto ───────────────────────────────────────────────
            label = Marker()
            label.header.frame_id = 'map'; label.header.stamp = zero
            label.ns = 'cz_zones'; label.id = mid; mid += 1
            label.type = Marker.TEXT_VIEW_FACING; label.action = Marker.ADD
            label.pose.position.x = dx
            label.pose.position.y = dy
            label.pose.position.z = 0.75
            label.pose.orientation.w = 1.0
            label.scale.z = 0.15
            label.color.r, label.color.g, label.color.b, label.color.a = cr, cg, cb, 1.0
            side = 'DIREITA' if direction == 'LEFT' else 'ESQUERDA'
            label.text = f'BLOQUEIA\n{side}'
            label.lifetime.sec = 0
            ma.markers.append(label)

            # ── Seta apontando para o lado BLOQUEADO ──────────────────────────
            # LEFT bloqueia à direita → seta aponta para -perp
            # RIGHT bloqueia à esquerda → seta aponta para +perp
            arrow_yaw = dyaw - math.pi / 2.0 if direction == 'LEFT' else dyaw + math.pi / 2.0
            acy, asy = math.cos(arrow_yaw * 0.5), math.sin(arrow_yaw * 0.5)
            arrow = Marker()
            arrow.header.frame_id = 'map'; arrow.header.stamp = zero
            arrow.ns = 'cz_zones'; arrow.id = mid; mid += 1
            arrow.type = Marker.ARROW; arrow.action = Marker.ADD
            arrow.pose.position.x = dx
            arrow.pose.position.y = dy
            arrow.pose.position.z = 0.40
            arrow.pose.orientation.z = asy
            arrow.pose.orientation.w = acy
            arrow.scale.x = 0.50   # comprimento
            arrow.scale.y = 0.07   # largura
            arrow.scale.z = 0.07
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = cr, cg, cb, 1.0
            arrow.lifetime.sec = 0
            ma.markers.append(arrow)

            # ── Cilindro semitransparente = raio de influência ────────────────
            cyl = Marker()
            cyl.header.frame_id = 'map'; cyl.header.stamp = zero
            cyl.ns = 'cz_zones'; cyl.id = mid; mid += 1
            cyl.type = Marker.CYLINDER; cyl.action = Marker.ADD
            cyl.pose.position.x = dx
            cyl.pose.position.y = dy
            cyl.pose.position.z = 0.05
            cyl.pose.orientation.w = 1.0
            cyl.scale.x = cyl.scale.y = self._COLOR_ZONE_RADIUS * 2.0
            cyl.scale.z = 0.02
            cyl.color.r, cyl.color.g, cyl.color.b, cyl.color.a = cr, cg, cb, 0.12
            cyl.lifetime.sec = 0
            ma.markers.append(cyl)

        # ── Parede virtual de entrada (células do grid) ───────────────────────
        info = self._slam_map.info
        res  = info.resolution
        for vidx, (gc, gr) in enumerate(self._virtual_wall_cells):
            wx, wy = self._g2w(info, gc, gr)
            cube = Marker()
            cube.header.frame_id = 'map'; cube.header.stamp = zero
            cube.ns = 'cz_vwall'; cube.id = vidx
            cube.type = Marker.CUBE; cube.action = Marker.ADD
            cube.pose.position.x = wx
            cube.pose.position.y = wy
            cube.pose.position.z = 0.20
            cube.pose.orientation.w = 1.0
            cube.scale.x = cube.scale.y = res * 1.1
            cube.scale.z = 0.40
            cube.color.r, cube.color.g, cube.color.b, cube.color.a = 1.0, 0.5, 0.0, 0.85
            cube.lifetime.sec = 0
            ma.markers.append(cube)

        self.color_zone_pub.publish(ma)

    def _publish_path_marker(self, path: List[Tuple[float, float]], chosen_idx: int):
        """
        Publica o path completo do A* no RViz (/maze_astar_path):
          • Linha cinza fina: path completo célula a célula (sem simplificação RDP)
          • Linha branca mais grossa: segmento do robô até o waypoint escolhido
          • Esfera ciano: waypoint escolhido
          • Esfera amarela: destino final (último ponto do path)
        """
        zero = RosTime(sec=0, nanosec=0)
        ma   = MarkerArray()

        # ── Limpa marcadores anteriores ───────────────────────────────────────
        clr = Marker()
        clr.header.frame_id = 'map'; clr.header.stamp = zero
        clr.ns = 'astar'; clr.id = 0; clr.action = Marker.DELETEALL
        ma.markers.append(clr)

        if not path:
            self.path_pub.publish(ma)
            return

        # ── Path completo — linha cinza fina ──────────────────────────────────
        full_line = Marker()
        full_line.header.frame_id = 'map'; full_line.header.stamp = zero
        full_line.ns = 'astar'; full_line.id = 1
        full_line.type = Marker.LINE_STRIP; full_line.action = Marker.ADD
        full_line.scale.x = 0.025
        full_line.color.r, full_line.color.g, full_line.color.b, full_line.color.a = \
            0.55, 0.55, 0.55, 0.8
        full_line.lifetime.sec = 0; full_line.pose.orientation.w = 1.0
        for wx, wy in path:
            pt = Point(); pt.x, pt.y, pt.z = wx, wy, 0.08
            full_line.points.append(pt)
        ma.markers.append(full_line)

        # ── Segmento até o waypoint escolhido — linha branca mais grossa ──────
        if chosen_idx > 0:
            seg_line = Marker()
            seg_line.header.frame_id = 'map'; seg_line.header.stamp = zero
            seg_line.ns = 'astar'; seg_line.id = 2
            seg_line.type = Marker.LINE_STRIP; seg_line.action = Marker.ADD
            seg_line.scale.x = 0.05
            seg_line.color.r, seg_line.color.g, seg_line.color.b, seg_line.color.a = \
                1.0, 1.0, 1.0, 1.0
            seg_line.lifetime.sec = 0; seg_line.pose.orientation.w = 1.0
            for wx, wy in path[:chosen_idx + 1]:
                pt = Point(); pt.x, pt.y, pt.z = wx, wy, 0.10
                seg_line.points.append(pt)
            ma.markers.append(seg_line)

        # ── Waypoint escolhido — esfera ciano ─────────────────────────────────
        wx_c, wy_c = path[chosen_idx]
        wp_sph = Marker()
        wp_sph.header.frame_id = 'map'; wp_sph.header.stamp = zero
        wp_sph.ns = 'astar'; wp_sph.id = 3
        wp_sph.type = Marker.SPHERE; wp_sph.action = Marker.ADD
        wp_sph.pose.position.x = wx_c; wp_sph.pose.position.y = wy_c
        wp_sph.pose.position.z = 0.18; wp_sph.pose.orientation.w = 1.0
        wp_sph.scale.x = wp_sph.scale.y = wp_sph.scale.z = 0.16
        wp_sph.color.r, wp_sph.color.g, wp_sph.color.b, wp_sph.color.a = 0.0, 1.0, 1.0, 1.0
        wp_sph.lifetime.sec = 0
        ma.markers.append(wp_sph)

        # ── Destino final — esfera amarela ────────────────────────────────────
        wx_f, wy_f = path[-1]
        goal_sph = Marker()
        goal_sph.header.frame_id = 'map'; goal_sph.header.stamp = zero
        goal_sph.ns = 'astar'; goal_sph.id = 4
        goal_sph.type = Marker.SPHERE; goal_sph.action = Marker.ADD
        goal_sph.pose.position.x = wx_f; goal_sph.pose.position.y = wy_f
        goal_sph.pose.position.z = 0.22; goal_sph.pose.orientation.w = 1.0
        goal_sph.scale.x = goal_sph.scale.y = goal_sph.scale.z = 0.20
        goal_sph.color.r, goal_sph.color.g, goal_sph.color.b, goal_sph.color.a = \
            1.0, 1.0, 0.0, 1.0
        goal_sph.lifetime.sec = 0
        ma.markers.append(goal_sph)

        self.path_pub.publish(ma)

    def _publish_wp_marker(self, wx: float, wy: float):
        zero = RosTime(sec=0, nanosec=0)
        ma   = MarkerArray()

        s = Marker()
        s.header.frame_id = 'map'; s.header.stamp = zero
        s.ns = 'wp'; s.id = self._wp_count
        s.type = Marker.SPHERE; s.action = Marker.ADD
        s.pose.position.x = wx; s.pose.position.y = wy; s.pose.position.z = 0.15
        s.pose.orientation.w = 1.0
        s.scale.x = s.scale.y = s.scale.z = 0.10
        t = min(1.0, self._wp_count / 40.0)
        s.color.r, s.color.g, s.color.b, s.color.a = t, 1.0 - t, 0.2, 1.0
        s.lifetime.sec = 0
        ma.markers.append(s)
        self._wp_trail.append((wx, wy))
        self._wp_count += 1

        if len(self._wp_trail) >= 2:
            trail = Marker()
            trail.header.frame_id = 'map'; trail.header.stamp = zero
            trail.ns = 'trail'; trail.id = 0
            trail.type = Marker.LINE_STRIP; trail.action = Marker.ADD
            trail.scale.x = 0.03
            trail.color.r, trail.color.g, trail.color.b, trail.color.a = 0.3, 0.6, 1.0, 0.8
            trail.lifetime.sec = 0; trail.pose.orientation.w = 1.0
            for px, py in self._wp_trail:
                pt = Point(); pt.x, pt.y, pt.z = px, py, 0.05
                trail.points.append(pt)
            ma.markers.append(trail)

        self.marker_pub.publish(ma)

    def _publish_frontier_markers(self, frontiers: List[Tuple[float, float]]):
        if not self.show_frontier_mrk:
            return
        zero = RosTime(sec=0, nanosec=0)
        ma   = MarkerArray()

        clr = Marker()
        clr.header.frame_id = 'map'; clr.header.stamp = zero
        clr.ns = 'frontiers'; clr.id = 0; clr.action = Marker.DELETEALL
        ma.markers.append(clr)

        for i, (fx, fy) in enumerate(frontiers):
            s = Marker()
            s.header.frame_id = 'map'; s.header.stamp = zero
            s.ns = 'frontiers'; s.id = i + 1
            s.type = Marker.SPHERE; s.action = Marker.ADD
            s.pose.position.x = fx; s.pose.position.y = fy; s.pose.position.z = 0.10
            s.pose.orientation.w = 1.0
            fade = max(0.25, 1.0 - i * 0.10)
            sz   = max(0.08, 0.18 - i * 0.01)
            s.color.r, s.color.g, s.color.b, s.color.a = 0.0, 0.85 * fade, 0.85 * fade, fade
            s.scale.x = s.scale.y = s.scale.z = sz
            s.lifetime.sec = 0
            ma.markers.append(s)

        self.frontier_pub.publish(ma)

    # ── loop principal (5 Hz) ─────────────────────────────────────────────────

    def _loop(self):
        if self.latest_scan is None or self.current_pose is None:
            return
        if self.state == self.S_SOLVED:
            return

        scan  = self.latest_scan
        am, ai, R = scan.angle_min, scan.angle_increment, scan.range_max
        front = sector_min(scan.ranges, am, ai,   0, 15, R)
        right = sector_min(scan.ranges, am, ai, -90, 30, R)
        left  = sector_min(scan.ranges, am, ai,  90, 30, R)

        if self.state == self.S_INIT:
            if self._nav_client.wait_for_server(timeout_sec=0.1):
                self.get_logger().info('Nav2 pronto.')
                self.state = self.S_STARTUP
            return

        if self.state == self.S_STARTUP:
            if front > self.startup_wall_d:
                t = Twist(); t.linear.x = self.startup_lin_spd
                self.cmd_pub.publish(t)
            else:
                self.cmd_pub.publish(Twist())
                rx, ry, ryaw = self.current_pose
                self._start_pos = (rx, ry)
                self.state = self.S_NAV

                # ── Parede virtual de entrada ─────────────────────────────────
                # Marca uma faixa de células letais perpendicular ao heading do
                # robô, logo atrás da posição atual. O A* trata essas células
                # exatamente como paredes reais — sem lógica especial.
                # A faixa fica 0.5 m atrás do robô e tem largura de ~1.5 m
                # (±half_w células perpendiculares ao yaw).
                if self._slam_map is not None:
                    info  = self._slam_map.info
                    res   = info.resolution
                    # Direção oposta ao heading (entrada fica atrás)
                    back_yaw = ryaw + math.pi
                    # Ponto central da faixa: 0.5 m atrás do robô
                    cx = rx + 0.5 * math.cos(back_yaw)
                    cy = ry + 0.5 * math.sin(back_yaw)
                    # Vetor perpendicular (lateral) ao heading
                    perp_x = -math.sin(ryaw)
                    perp_y =  math.cos(ryaw)
                    # Largura da faixa em células (±half_w de cada lado)
                    half_w = max(1, int(0.8 / res))
                    for k in range(-half_w, half_w + 1):
                        wx = cx + k * perp_x * res
                        wy = cy + k * perp_y * res
                        gc, gr = self._w2g(info, wx, wy)
                        self._virtual_wall_cells.append((gc, gr))
                    self.get_logger().info(
                        f'Solver ativo. Exclusão em ({rx:.2f},{ry:.2f}) | '
                        f'Parede virtual de entrada: {len(self._virtual_wall_cells)} células '
                        f'em ({cx:.2f},{cy:.2f}) yaw={math.degrees(ryaw):.1f}°'
                    )
                    self._publish_color_zone_markers()
                else:
                    self.get_logger().info(
                        f'Solver ativo. Exclusão em ({rx:.2f},{ry:.2f}) | '
                        f'Mapa não disponível — parede virtual não inserida.'
                    )
            return

        if front > self.open_thresh and right > self.open_thresh and left > self.open_thresh:
            self._cancel_goal()
            self.state = self.S_SOLVED
            self.get_logger().info('Labirinto resolvido!')
            return

        now = self.get_clock().now()
        elapsed = ((now - self._last_replan).nanoseconds / 1e9
                   if self._last_replan else float('inf'))

        if elapsed >= self.replan_interval:
            wp = self._plan_next_wp()
            if wp:
                self._send_wp(*wp)
            else:
                self.get_logger().warn('Sem waypoint — aguardando mapa...')

        rx, ry, _ = self.current_pose
        dist = (math.hypot(rx - self._current_wp[0], ry - self._current_wp[1])
                if self._current_wp else 0.0)
        s = String()
        s.data = (f'[{self.state}] dist={dist:.2f}m next={max(0.0, self.replan_interval-elapsed):.1f}s'
                  f' | F={front:.2f} R={right:.2f} L={left:.2f}'
                  f' | ({rx:.2f},{ry:.2f})')
        self.status_pub.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = MazeNav2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._goal_handle is not None:
            node._goal_handle.cancel_goal_async()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
