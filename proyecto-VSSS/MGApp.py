# ======================================================================
# Plataforma desarrollada para utilizarse en el Torneo Zacatecano de
# Fútbol Robótico en la categoría VSSS, organizado por el Consejo
# Zacatecano de Ciencia, Tecnología e Innovación (COZCYT) y el Club
# de Robótica RobTai-UAZ
# Desarrollada por: M. en C. José Manuel Mercado Blanco
# MGApp: combina MApp.py + GApp.py. Compatible con Windows y macOS.
#
# Mejoras sobre MApp.py integradas desde GApp.py:
#   - Calibración per-robot: ROBOT_FRONT_SIGN y ROBOT_PWM_CALIBRATION
#   - Control PWM estable (alinea primero, luego avanza)
#   - Filtros adicionales de detección: filter_bases_by_area,
#     filter_rival_bases_near_own, choose_role_markers
#   - Detección de pelota con circularidad y fill_ratio
#   - Dos roles activos: portero y delantero con cobertura defensiva
#   - search_pwm: giro de búsqueda cuando un robot se pierde
#   - keep_out_goal_areas: evita entrar al área propia
#   - Botones: "Auto colores" y "Auto camara"
#   - MEMORY_FRAMES/MAX_LOST_FRAMES más largos para mayor robustez
# ======================================================================

import cv2
import numpy as np
import json
import os
import sys
import time
import serial
import threading
import queue
from serial.tools import list_ports
from PIL import Image, ImageTk
import tkinter as tk
from tkinter import ttk, messagebox
from urllib.parse import urlsplit


class VideoSource:
    """Apertura de fuentes de video locales y de red."""

    @staticmethod
    def open_video_source(index=0, source='local', url=''):
        """Abre una webcam física/virtual o una URL de video del teléfono."""
        if source == 'iphone':
            url = url.strip()
            parts = urlsplit(url)
            if parts.scheme.lower() not in ('http', 'https', 'rtsp', 'rtsps') or not parts.hostname:
                raise ValueError('Introduce una URL de video HTTP(S) o RTSP válida. '
                                 'Para DroidCam por USB selecciona Local / iPhone USB.')
            # Los timeouts se establecen al abrir: set() posterior no los aplica.
            cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG, [
                cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000,
                cv2.CAP_PROP_READ_TIMEOUT_MSEC, 2000,
            ])
            if not cap.isOpened():
                cap.release()
                raise RuntimeError('No se pudo abrir el video del iPhone. Comprueba la URL '
                                   'de transmisión, la app del teléfono y la red.')
        else:
            if sys.platform == 'win32':
                backends = (cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY)
            elif sys.platform == 'darwin':
                backends = (cv2.CAP_AVFOUNDATION, cv2.CAP_ANY)
            else:
                backends = (cv2.CAP_ANY,)
            cap = None
            for backend in backends:
                candidate = None
                try:
                    candidate = cv2.VideoCapture(int(index), backend)
                    if candidate.isOpened():
                        candidate.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                        candidate.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                        candidate.set(cv2.CAP_PROP_FPS, 30)
                        ok, first = candidate.read()
                        if ok and first is not None:
                            cap = candidate
                            break
                except cv2.error:
                    # Algunos drivers lanzan error en vez de devolver False.
                    pass
                finally:
                    if candidate is not None and candidate is not cap:
                        candidate.release()
            if cap is None:
                raise RuntimeError(f'No se pudo leer la cámara {index}. Prueba otro índice, '
                                   'abre DroidCam y habilita el acceso a la cámara para '
                                   'aplicaciones de escritorio en Windows.')
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap


# Alias públicos para conservar compatibilidad con el código existente.
open_video_source = VideoSource.open_video_source

# ============================================================
# NOMBRES DE LOS ROBOTS
# ============================================================
robot_POR = "POR"
robot_DEL = "DEL"
ACTIVE_ROLES = (robot_POR, robot_DEL)

# ============================================================
# CONFIGURACIONES GLOBALES
# ============================================================
CLEARANCE_DIST           = 80
MEMORY_FRAMES            = 35    # frames que se recuerda posición perdida
MAX_LOST_FRAMES          = 18    # frames antes de congelar posición estimada
MAX_LOST_FRAMES_RECOVERY = 28    # frames para activar giro de búsqueda

SERIAL_BAUDRATE = 921600
SERIAL_TIMEOUT  = 0.01
SERIAL_REFRESH_SECONDS = 0.10  # Menor que TIMEOUT_MS=250 del esclavo.

FIELD_ASPECT_RATIO = (150, 130)
BASE_RESOLUTION    = (750, 650)

# ── Parámetros PWM ────────────────────────────────────────────
PWM_BASE_SPEED           = 115
PWM_TURN_FACTOR_MOVING   = 0.85
PWM_TURN_FACTOR_IN_PLACE = 1.25
PWM_MAX                  = 170
ANGLE_DEADBAND           = 12
ANGLE_TURN_ONLY          = 38
GOAL_AREA_DEPTH_RATIO    = 0.133  # 20cm = rect 15cm + arco 12.5r desde 7.5cm (Sec. 2.4)
GOAL_AREA_HEIGHT_RATIO   = 0.538  # 70cm / 130cm (Sec. 2.4)
FIELD_SAFE_MARGIN        = 55

# ── Orientación de tapa por robot ─────────────────────────────
#  1  = marcadores apuntan al frente real del robot
# -1  = marcadores apuntan hacia atrás; invierte orientación 180°
ROBOT_FRONT_SIGN = {
    robot_POR: 1,
    robot_DEL: 1,
}

# ── Calibración PWM por motor ─────────────────────────────────
# Ajusta si un motor empuja más que el otro (valores > 1 = más fuerza)
ROBOT_PWM_CALIBRATION = {
    robot_POR: {'left': 1.00, 'right': 1.00},
    robot_DEL: {'left': 1.00, 'right': 1.00},
}

# ── Máquina de estados de atacante ────────────────────────────
ROBOT_STATE = {robot_DEL: 'ATAQUE'}

# ----------------------------------------------------------
# MAPEO DE COLORES -> ROL
# ----------------------------------------------------------
# Pares de marcadores → rol, según la Ficha de Uniformes oficial VSSS 2026.
# Todos los robots del equipo comparten el mismo color de BASE GRANDE.
#
# Uniforme AMARILLO (base = amarillo):
#   DEL → verde + rosa   |  POR → azul + rosa
#
# Uniforme AZUL (base = azul):
#   DEL → verde + rosa   |  POR → amarillo + rojo
class TeamVision:
    """Detección de pelota, bases y marcadores de los equipos."""

    @staticmethod
    def get_role_color_pairs(my_color):
        """Retorna el dict de pares de marcadores → rol para el uniforme activo."""
        if my_color == 'amarillo':
            return {
                ('rosa',     'verde'): robot_DEL,
                ('azul',     'rosa'):  robot_POR,
            }
        else:   # azul
            return {
                ('rosa',     'verde'): robot_DEL,
                ('amarillo', 'rojo'):  robot_POR,
            }


    @staticmethod
    def lab_polar_mask(bgr, l_min, l_max, r_min, r_max, t_min, t_max, _precomp=None):
        """Máscara binaria en espacio Lab polar (L, Chroma, Ángulo)."""
        if _precomp is not None:
            l_ch, mag, angle = _precomp
        else:
            lab  = cv2.cvtColor(bgr, cv2.COLOR_BGR2Lab)
            l_ch = lab[:, :, 0]
            a    = lab[:, :, 1].astype(np.float32) - 128.0
            b    = lab[:, :, 2].astype(np.float32) - 128.0
            mag, angle = cv2.cartToPolar(a, b, angleInDegrees=True)
        mask_l = cv2.inRange(l_ch, int(l_min), int(l_max))
        mask_r = cv2.inRange(mag, float(r_min), float(r_max))
        if t_min <= t_max:
            mask_t = cv2.inRange(angle, float(t_min), float(t_max))
        else:
            mask_t = cv2.bitwise_or(
                cv2.inRange(angle, float(t_min), 360.0),
                cv2.inRange(angle, 0.0, float(t_max))
            )
        return cv2.bitwise_and(mask_l, cv2.bitwise_and(mask_r, mask_t))


    @staticmethod
    def detect_ball(bgr, lab_range, _precomp=None, last_pos=None):
        """
        Detecta la pelota con Lab polar + filtros de circularidad y fill ratio.

        last_pos : última posición DETECTADA (no predicción Kalman).
                   Se usa como desempate suave entre candidatos igualmente válidos.
                   No debe usarse la predicción Kalman aquí para evitar bucles de
                   realimentación que desplazan la detección hacia posiciones erróneas.
        """
        mask = lab_polar_mask(bgr, **lab_range, _precomp=_precomp)

        h, w = mask.shape[:2]
        border = 20
        mask[:border, :]   = 0
        mask[h-border:, :] = 0
        mask[:, :border]   = 0
        mask[:, w-border:] = 0

        # Solo MORPH_OPEN: elimina ruido puntual sin crear blobs falsos.
        # SIN GaussianBlur (desplaza centroides en máscaras binarias).
        # SIN MORPH_CLOSE (une manchas distantes y genera pelotas fantasma).
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _MORPH_KERNEL_3x3)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        for c in contours:
            area = cv2.contourArea(c)
            if area < 40 or area > 1200:
                continue
            (x, y), radius = cv2.minEnclosingCircle(c)
            if radius < 4 or radius > 22:
                continue
            perim = cv2.arcLength(c, True)
            if perim <= 0:
                continue
            circularity = 4 * np.pi * area / (perim * perim)
            fill_ratio  = area / (np.pi * radius * radius)
            if circularity < 0.45 or fill_ratio < 0.35:
                continue
            # Centroide por momentos: más preciso que minEnclosingCircle
            M = cv2.moments(c)
            if M["m00"] > 0:
                x = M["m10"] / M["m00"]
                y = M["m01"] / M["m00"]
            score = area * circularity * fill_ratio
            # Bonus pequeño por cercanía a la última posición detectada real.
            # Máximo +40 para desempatar sin contaminar la elección principal.
            if last_pos is not None:
                dist   = np.hypot(x - last_pos[0], y - last_pos[1])
                score += max(0.0, 40.0 - dist)
            if best is None or score > best[0]:
                best = (score, int(x), int(y))
        return (best[1], best[2]) if best else None


    @staticmethod
    def get_rects(bgr, lab_range, min_a=15, max_a=5000, _precomp=None):
        mask = lab_polar_mask(bgr, **lab_range, _precomp=_precomp)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _MORPH_KERNEL_3x3)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return [cv2.minAreaRect(c) for c in contours if min_a < cv2.contourArea(c) < max_a]


    @staticmethod
    def get_closest_marker_center(base_center, markers, radius):
        closest, min_dist = None, radius
        for m in markers:
            (mx, my), _, _ = m
            d = np.hypot(mx - base_center[0], my - base_center[1])
            if d < min_dist:
                min_dist = d
                closest  = np.array([mx, my])
        return closest


    @staticmethod
    def filter_bases_by_area(bases, min_area=250, min_side=12):
        """Evita que marcadores chicos se tomen como bases de robot."""
        filtered = []
        for base in bases:
            (_, _), (w, h), _ = base
            if w * h >= min_area and min(w, h) >= min_side:
                filtered.append(base)
        return filtered


    @staticmethod
    def filter_rival_bases_near_own(rival_bases, own, min_dist=55):
        """Descarta falsos rivales provocados por marcadores dentro de robots propios."""
        own_positions = [r['pos'] for r in own.values() if r is not None]
        filtered = []
        for base in rival_bases:
            (bx, by), _, _ = base
            if not any(np.hypot(bx - ox, by - oy) < min_dist for ox, oy in own_positions):
                filtered.append(base)
        return filtered


    @staticmethod
    def filter_out_base_markers(bases, markers_dict, min_dist=10):
        """
        Elimina marcadores cuyo centro esté a menos de min_dist del centro de alguna base
        Y cuya área sea >= 40% del área de esa base (los stickers pequeños sobreviven).
        """
        filtered = {}
        for color_name, marker_list in markers_dict.items():
            filtered[color_name] = []
            for m in marker_list:
                (mx, my), (mw, mh), _ = m
                keep = True
                for base in bases:
                    (bx, by), (bw, bh), _ = base
                    if np.hypot(mx - bx, my - by) < min_dist:
                        if (mw * mh) >= (bw * bh) * 0.40:
                            keep = False
                            break
                if keep:
                    filtered[color_name].append(m)
        return filtered


    @staticmethod
    def choose_role_markers(closest_of_color, role_pairs=None):
        """
        Elige el par válido de marcadores más cercano a la base.
        Evita que un tercer color cercano rompa la identificación del rol.
        closest_of_color: {color: (idx, pos, dist)}
        role_pairs: dict par→rol (por defecto usa ROLE_COLOR_PAIRS global)
        """
        if role_pairs is None:
            role_pairs = ROLE_COLOR_PAIRS
        best = None
        for pair, role in role_pairs.items():
            if pair[0] not in closest_of_color or pair[1] not in closest_of_color:
                continue
            dist_sum = closest_of_color[pair[0]][2] + closest_of_color[pair[1]][2]
            if best is None or dist_sum < best[0]:
                best = (dist_sum, pair, role)
        if best is None:
            return None, None
        return best[1], best[2]


    @staticmethod
    def identify_team(bgr, colors, my_color, _precomp=None):
        """
        Identifica los dos roles propios POR/DEL y los robots rivales.
        Usa espacio Lab polar. Integra filtros de GApp para mayor robustez.
        """
        if my_color == 'azul':
            my_base_range    = colors['azul']
            rival_base_range = colors['amarillo']
        else:
            my_base_range    = colors['amarillo']
            rival_base_range = colors['azul']

        # Detectar bases: rivales con min_a mayor para evitar confusión con stickers
        my_bases_raw    = get_rects(bgr, my_base_range,    50,  5000, _precomp=_precomp)
        rival_bases_raw = get_rects(bgr, rival_base_range, 200, 5000, _precomp=_precomp)

        # Filtrar por área mínima y lado mínimo
        my_bases    = filter_bases_by_area(my_bases_raw,    min_area=300, min_side=12)
        rival_bases = filter_bases_by_area(rival_bases_raw, min_area=650, min_side=18)

        # Pares de colores → rol según el uniforme activo del torneo
        role_pairs = get_role_color_pairs(my_color)

        # Detectar marcadores de colores (incluye 'rojo' si está calibrado)
        markers_p = get_rects(bgr, colors['rosa'],  15, 2000, _precomp=_precomp)
        markers_g = get_rects(bgr, colors['verde'], 15, 2000, _precomp=_precomp)
        markers_b = get_rects(bgr, colors['azul'],  15, 2000, _precomp=_precomp)
        markers_r = get_rects(bgr, colors['rojo'],  15, 2000, _precomp=_precomp) \
                    if 'rojo' in colors else []

        available_markers = {
            'rosa':     markers_p,
            'verde':    markers_g,
            'azul':     markers_b,
            'rojo':     markers_r,
            'amarillo': get_rects(bgr, colors['amarillo'], 15, 2000, _precomp=_precomp)
                        if my_color == 'azul' else [],
        }

        # Filtrar marcadores: excluir los que están sobre bases propias O rivales
        filtered_my    = filter_out_base_markers(my_bases + rival_bases, available_markers)
        filtered_rival = filter_out_base_markers(rival_bases, available_markers)

        # ── Robots propios ────────────────────────────────────────
        own = {role: None for role in ACTIVE_ROLES}
        used_markers = set()

        for base in my_bases:
            (bx, by), (bw, bh), _ = base
            base_center = np.array([bx, by])
            radius      = max(bw, bh) * 1.5

            # Marcador más cercano de cada color (3-tupla: idx, pos, dist)
            closest_of_color = {}
            for color_name, marker_list in filtered_my.items():
                candidate, min_d = None, radius
                for i, m in enumerate(marker_list):
                    (mx, my2), _, _ = m
                    d = np.hypot(mx - bx, my2 - by)
                    if d < min_d:
                        min_d     = d
                        candidate = (i, np.array([mx, my2]), d)
                if candidate is not None:
                    closest_of_color[color_name] = candidate

            pair, role = choose_role_markers(closest_of_color, role_pairs)
            if pair is None:
                continue

            idx1, idx2 = closest_of_color[pair[0]][0], closest_of_color[pair[1]][0]
            if (pair[0], idx1) in used_markers or (pair[1], idx2) in used_markers:
                continue
            used_markers.add((pair[0], idx1))
            used_markers.add((pair[1], idx2))

            pos1 = closest_of_color[pair[0]][1]
            pos2 = closest_of_color[pair[1]][1]
            mean_pos     = (pos1 + pos2) / 2.0
            front_vector = mean_pos - base_center
            angle_deg    = (np.degrees(np.arctan2(front_vector[1], front_vector[0])) + 360) % 360

            if ROBOT_FRONT_SIGN.get(role, 1) == -1:
                angle_deg = (angle_deg + 180) % 360

            own[role] = {'pos': (int(bx), int(by)), 'angle': angle_deg}

        # ── Robots rivales ────────────────────────────────────────
        # Filtrar falsos rivales que caen cerca de robots propios ya identificados
        rival_bases    = filter_rival_bases_near_own(rival_bases, own)
        filtered_rival = filter_out_base_markers(rival_bases, available_markers)

        rivals = []
        for base in rival_bases:
            (bx, by), (bw, bh), _ = base
            base_center = np.array([bx, by])
            radius      = max(bw, bh) * 1.5

            closest_marker, min_dist_m = None, radius
            for color_name, marker_list in filtered_rival.items():
                for m in marker_list:
                    (mx, my2), _, _ = m
                    d = np.hypot(mx - bx, my2 - by)
                    if d < min_dist_m:
                        min_dist_m    = d
                        closest_marker = np.array([mx, my2])

            if closest_marker is not None:
                fv    = base_center - closest_marker
                angle = (np.degrees(np.arctan2(fv[1], fv[0])) + 360) % 360
                rivals.append({'pos': (int(bx), int(by)), 'angle': angle})

        return own, rivals


# Alias públicos para conservar compatibilidad con el código existente.
get_role_color_pairs = TeamVision.get_role_color_pairs
lab_polar_mask = TeamVision.lab_polar_mask
detect_ball = TeamVision.detect_ball
get_rects = TeamVision.get_rects
get_closest_marker_center = TeamVision.get_closest_marker_center
filter_bases_by_area = TeamVision.filter_bases_by_area
filter_rival_bases_near_own = TeamVision.filter_rival_bases_near_own
filter_out_base_markers = TeamVision.filter_out_base_markers
choose_role_markers = TeamVision.choose_role_markers
identify_team = TeamVision.identify_team

# Alias estático para compatibilidad con código externo (se usa my_color='azul' por defecto)
ROLE_COLOR_PAIRS = get_role_color_pairs('azul')

# ── Colores de visualización (BGR) ─────────────────────────────
COLOR_POR_OWN  = (0, 255, 0)
COLOR_DEL_OWN  = (0, 255, 0)
COLOR_RIVAL    = (0, 0, 255)
COLOR_BALL     = (0, 165, 255)
COLOR_HUD_BG   = (0, 0, 0)
COLOR_TARGET   = (255, 0, 255)
COLOR_REPULSION= (0, 0, 255)

# ============================================================
# CONSTANTES PID
# ============================================================
Kp_ang = 0.8
Ki_ang = 0.01
Kd_ang = 0.1
DT     = 1.0 / 30.0

_MORPH_KERNEL_3x3 = np.ones((3, 3), np.uint8)

class MotionController:
    """Control angular, PWM y navegación segura."""

    @staticmethod
    def control_pid_angular(error, dt, integral, prev_error):
        integral = max(min(integral + error * dt, 50), -50)
        derivada = (error - prev_error) / dt if dt > 0 else 0
        output   = Kp_ang * error + Ki_ang * integral + Kd_ang * derivada
        return output, integral, error


    @staticmethod
    def aplicar_repulsion(mi_pos, target, obstaculos, radio_colision, fuerza_max, dist_to_target=0):
        tx, ty = target
        rx, ry = mi_pos
        factor = max(0.0, min(1.0, dist_to_target / 120.0)) if dist_to_target > 0 else 1.0
        for ox, oy in obstaculos:
            dist = np.hypot(ox - rx, oy - ry)
            if 0 < dist < radio_colision:
                dx, dy = rx - ox, ry - oy
                mag    = np.hypot(dx, dy) + 1e-8
                fuerza = min((radio_colision - dist) / radio_colision * fuerza_max * factor, 30)
                tx += (dx / mag) * fuerza
                ty += (dy / mag) * fuerza
        return int(tx), int(ty)


    @staticmethod
    def aplicar_repulsion_bordes(tx, ty, rx, ry, frame_w, frame_h, borde, fuerza):
        fx = fy = 0
        if rx < borde:           fx += (1.0 - rx / borde) * fuerza
        if rx > frame_w - borde: fx -= (1.0 - (frame_w - rx) / borde) * fuerza
        if ry < borde:           fy += (1.0 - ry / borde) * fuerza
        if ry > frame_h - borde: fy -= (1.0 - (frame_h - ry) / borde) * fuerza
        return max(0, min(frame_w, int(tx + fx))), max(0, min(frame_h, int(ty + fy)))


    @staticmethod
    def compute_pwm(turn, speed):
        """PID angular directo → PWM izq/der."""
        if speed == 0:
            if turn != 0:
                left  = int(np.clip(-turn * 2.5, -255, 255))
                right = int(np.clip( turn * 2.5, -255, 255))
                return f"{left},{right}"
            return "0,0"
        base  = 150
        left  = int(np.clip(base - turn * 1.5, -255, 255))
        right = int(np.clip(base + turn * 1.5, -255, 255))
        return f"{left},{right}"


    @staticmethod
    def compute_pwm_stable(error_ang, dist, stop_distance=28):
        """Alinea primero si el error es grande, luego avanza. Más estable."""
        if dist <= stop_distance:
            return "0,0"
        if abs(error_ang) > ANGLE_TURN_ONLY:
            turn_pwm = int(np.clip(error_ang * 1.4, -PWM_MAX, PWM_MAX))
            return f"{-turn_pwm},{turn_pwm}"
        turn = 0 if abs(error_ang) <= ANGLE_DEADBAND else int(np.clip(error_ang * 0.65, -42, 42))
        base = PWM_BASE_SPEED
        left  = int(np.clip(base - turn * PWM_TURN_FACTOR_MOVING, -PWM_MAX, PWM_MAX))
        right = int(np.clip(base + turn * PWM_TURN_FACTOR_MOVING, -PWM_MAX, PWM_MAX))
        return f"{left},{right}"


    @staticmethod
    def apply_robot_calibration(cmd, role):
        """Escala PWM por motor para compensar desequilibrios físicos."""
        try:
            left, right = [int(x) for x in cmd.split(',')]
        except Exception:
            return cmd
        cal   = ROBOT_PWM_CALIBRATION.get(role, {'left': 1.0, 'right': 1.0})
        left  = int(np.clip(left  * cal['left'],  -PWM_MAX, PWM_MAX))
        right = int(np.clip(right * cal['right'], -PWM_MAX, PWM_MAX))
        return f"{left},{right}"


    @staticmethod
    def target_error_and_dist(robot, target):
        rx, ry  = robot['pos']
        tx, ty  = target
        dist    = np.hypot(tx - rx, ty - ry)
        ang_obj = (np.degrees(np.arctan2(ty - ry, tx - rx)) + 360) % 360
        error   = ((ang_obj - robot['angle'] + 180) % 360) - 180
        return error, dist


    @staticmethod
    def drive_to_point_stable(robot, target, stop_distance=28):
        if robot is None or target is None:
            return "0,0", None
        error, dist = target_error_and_dist(robot, target)
        return compute_pwm_stable(error, dist, stop_distance), target


    @staticmethod
    def drive_to_point_safe(robot, target, frame_w, frame_h, stop_distance=28):
        if robot is None or target is None:
            return "0,0", None
        rx, ry = robot['pos']
        tx, ty = target
        near_wall = ((rx < FIELD_SAFE_MARGIN and tx <= rx) or
                     (rx > frame_w - FIELD_SAFE_MARGIN and tx >= rx) or
                     (ry < FIELD_SAFE_MARGIN and ty <= ry) or
                     (ry > frame_h - FIELD_SAFE_MARGIN and ty >= ry))
        if near_wall:
            center = (frame_w // 2, frame_h // 2)
            cmd, _ = drive_to_point_stable(robot, center, stop_distance=9999)
            return cmd, center
        return drive_to_point_stable(robot, target, stop_distance)


    @staticmethod
    def search_pwm(role):
        """Giro suave diferenciado por rol cuando el robot no es visto."""
        if role == robot_POR: return compute_pwm(-12, 0)
        return compute_pwm(14, 0)


    @staticmethod
    def clamp_point(point, w, h, margin=25):
        x, y = point
        return (int(np.clip(x, margin, w - margin)),
                int(np.clip(y, margin, h - margin)))


    @staticmethod
    def keep_out_goal_areas(point, own_goal, opp_goal, w, h, margin=12):
        """Mantiene al delantero fuera del área propia; permite atacar el arco rival."""
        x, y       = point
        area_depth = int(w * GOAL_AREA_DEPTH_RATIO)
        area_half  = int(h * GOAL_AREA_HEIGHT_RATIO / 2)
        goal_y     = h // 2
        in_goal_y  = goal_y - area_half <= y <= goal_y + area_half
        if in_goal_y:
            if own_goal[0] == 0 and x < area_depth:
                x = area_depth + margin
            elif own_goal[0] != 0 and x > w - area_depth:
                x = w - area_depth - margin
        return clamp_point((x, y), w, h)


    @staticmethod
    def attack_geometry(ball, opp_goal, behind_distance, push_distance=95):
        bx, by = ball
        ox, oy = opp_goal
        to_x   = bx - ox;  to_y = by - oy
        mag    = np.hypot(to_x, to_y)
        if mag <= 0:
            return ball, ball
        behind = (bx + (to_x / mag) * behind_distance,
                  by + (to_y / mag) * behind_distance)
        push   = (bx - (to_x / mag) * push_distance,
                  by - (to_y / mag) * push_distance)
        return behind, push


    @staticmethod
    def update_attacker_state(role, robot, ball, behind_target):
        error_to_ball, dist_to_ball = target_error_and_dist(robot, ball)
        _, dist_to_behind           = target_error_and_dist(robot, behind_target)
        state = ROBOT_STATE.get(role, 'IR_DETRAS')

        if dist_to_ball < 55 and abs(error_to_ball) < 28:
            state = 'EMPUJAR'
        elif dist_to_behind < 42:
            state = 'ALINEAR'
        elif state == 'EMPUJAR' and dist_to_ball < 90:
            state = 'EMPUJAR'
        else:
            state = 'IR_DETRAS'

        ROBOT_STATE[role] = state
        return state


# Alias públicos para conservar compatibilidad con el código existente.
control_pid_angular = MotionController.control_pid_angular
aplicar_repulsion = MotionController.aplicar_repulsion
aplicar_repulsion_bordes = MotionController.aplicar_repulsion_bordes
compute_pwm = MotionController.compute_pwm
compute_pwm_stable = MotionController.compute_pwm_stable
apply_robot_calibration = MotionController.apply_robot_calibration
target_error_and_dist = MotionController.target_error_and_dist
drive_to_point_stable = MotionController.drive_to_point_stable
drive_to_point_safe = MotionController.drive_to_point_safe
search_pwm = MotionController.search_pwm
clamp_point = MotionController.clamp_point
keep_out_goal_areas = MotionController.keep_out_goal_areas
attack_geometry = MotionController.attack_geometry
update_attacker_state = MotionController.update_attacker_state

# ============================================================
# FUNCIONES AUXILIARES JSON
# ============================================================
class JsonStorage:
    """Persistencia de configuraciones JSON."""

    @staticmethod
    def load_json(file_path):
        try:
            with open(file_path, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"Error cargando {file_path}: {e}")
            return None


    @staticmethod
    def save_json(file_path, data):
        try:
            with open(file_path, 'w') as f:
                json.dump(data, f, indent=2)
            return True
        except Exception as e:
            print(f"Error guardando {file_path}: {e}")
            return False


# Alias públicos para conservar compatibilidad con el código existente.
load_json = JsonStorage.load_json
save_json = JsonStorage.save_json


# ============================================================
# UTILIDADES DE IMAGEN
# ============================================================
class ImageProcessing:
    """Transformaciones y ajustes de imagen."""

    def __init__(self):
        self._undistort_cache = {}
        self._warp_cache = {}

    def order_points_clockwise(self, pts):
        rect = np.zeros((4, 2), dtype="float32")
        pts  = np.array(pts, dtype="float32")
        s    = pts.sum(axis=1)
        diff = np.diff(pts, axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]
        return rect


    def init_kalman(self):
        kf = cv2.KalmanFilter(6, 2, 0)
        kf.transitionMatrix = np.array([
            [1,0,1,0,0.5,0],[0,1,0,1,0,0.5],
            [0,0,1,0,1,0],[0,0,0,1,0,1],
            [0,0,0,0,1,0],[0,0,0,0,0,1]
        ], np.float32)
        kf.measurementMatrix   = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0]], np.float32)
        kf.processNoiseCov     = np.eye(6, dtype=np.float32) * 1e-3
        kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-1
        return kf


    def undistort_frame(self, frame, camera_matrix, dist_coeffs):
        if camera_matrix is not None and dist_coeffs is not None:
            h, w = frame.shape[:2]
            key = (id(camera_matrix), w, h)
            if key not in self._undistort_cache:
                newcameramtx, _ = cv2.getOptimalNewCameraMatrix(
                    camera_matrix, dist_coeffs, (w, h), 1, (w, h))
                self._undistort_cache[key] = newcameramtx
            return cv2.undistort(frame, camera_matrix, dist_coeffs, None, self._undistort_cache[key])
        return frame


    def warp_image(self, frame, src_pts, output_size=BASE_RESOLUTION):
        key = (tuple(map(tuple, src_pts)), output_size)
        if key not in self._warp_cache:
            sp  = self.order_points_clockwise(src_pts)
            ul, ur, lr, ll = sp
            src = np.array([ul, ur, lr, ll], dtype="float32")
            nW, nH = output_size
            dst = np.array([[0,0],[nW-1,0],[nW-1,nH-1],[0,nH-1]], dtype="float32")
            self._warp_cache[key] = (cv2.getPerspectiveTransform(src, dst), nW, nH)
        M, nW, nH = self._warp_cache[key]
        return cv2.warpPerspective(frame, M, (nW, nH), flags=cv2.INTER_CUBIC), nW, nH


    def adjust_image(self, frame, brightness=128, contrast=128, saturation=128, exposure=-6):
        """Ajuste de imagen en software (AVFoundation ignora CAP_PROP_BRIGHTNESS, etc.)."""
        ev_delta   = exposure - (-6)
        exp_factor = float(np.clip(2.0 ** ev_delta, 0.1, 8.0))
        alpha = (contrast / 128.0) * exp_factor
        beta  = brightness - 128.0
        frame = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)
        if abs(saturation - 128) > 2:
            s_factor = saturation / 128.0
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV).astype(np.float32)
            hsv[:, :, 1] = np.clip(hsv[:, :, 1] * s_factor, 0, 255)
            frame = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)
        return frame


image_processing = ImageProcessing()

# Alias públicos para conservar compatibilidad con el código existente.
order_points_clockwise = image_processing.order_points_clockwise
init_kalman = image_processing.init_kalman
undistort_frame = image_processing.undistort_frame
warp_image = image_processing.warp_image
adjust_image = image_processing.adjust_image


_undistort_cache: dict = image_processing._undistort_cache


_warp_cache: dict = image_processing._warp_cache


# ============================================================
# FUNCIONES DE REPULSIÓN Y PREDICTOR
# ============================================================


class RobotPredictor:
    def __init__(self):
        self.last_pos   = None
        self.velocity   = (0, 0)
        self.lost_frames= 0
        self.filtered_pos = None

    def update(self, detected_pos):
        if detected_pos is not None:
            x, y = detected_pos
            if self.last_pos is not None:
                vx = x - self.last_pos[0]
                vy = y - self.last_pos[1]
                alpha = 0.7
                self.velocity = (alpha*vx + (1-alpha)*self.velocity[0],
                                 alpha*vy + (1-alpha)*self.velocity[1])
            self.last_pos     = (x, y)
            self.lost_frames  = 0
            self.filtered_pos = (x, y)
        else:
            self.lost_frames += 1
            if self.last_pos is not None and self.lost_frames <= MAX_LOST_FRAMES:
                vx, vy = self.velocity
                self.filtered_pos = (int(self.last_pos[0] + vx * self.lost_frames),
                                     int(self.last_pos[1] + vy * self.lost_frames))
            else:
                self.filtered_pos = None

    def is_recovery(self):
        return self.lost_frames > MAX_LOST_FRAMES_RECOVERY


# ============================================================
# ESTRATEGIAS DE JUEGO
# ============================================================
# ----------------------------------------------------------
# PORTERO – BLOQUEO / DESPEJE con PID angular
# ----------------------------------------------------------
class GoalkeeperStrategy:
    """Estrategia con memoria PID propia; invocable como la función original."""

    def __call__(self, robot, ball, own_goal, opp_goal, w, h):
        """
        BLOQUEO : se posiciona en x=def_x interceptando la línea portería→pelota.
                  El clip Y usa la boca real del arco (GOAL_AREA_HEIGHT_RATIO).
        DESPEJE : ≥ 2 frames con pelota en zona Y < 95 px → se pone detrás y despeja
                  hacia la banda lateral más cercana para alejar el peligro.
        """
        if robot is None:
            return "0,0", None

        rx, ry = robot['pos']
        gx, gy = own_goal
        ox, oy = opp_goal

        goal_half_h = int(h * GOAL_AREA_HEIGHT_RATIO / 2)
        goal_y_min  = h // 2 - goal_half_h
        goal_y_max  = h // 2 + goal_half_h
        area_depth  = int(w * GOAL_AREA_DEPTH_RATIO)

        # POR confinado al área de portería (Sec. 2.4): X ≤ area_depth, Y dentro de boca de arco
        if gx == 0:
            def_x   = 70;  zona_lim = area_depth
            en_zona = lambda bx: bx <= zona_lim
            clamp_x = lambda x: int(np.clip(x, 20, zona_lim))
        else:
            def_x   = w - 70;  zona_lim = w - area_depth
            en_zona = lambda bx: bx >= zona_lim
            clamp_x = lambda x: int(np.clip(x, zona_lim, w - 20))

        if not hasattr(self, '_st'):
            self._st = {
                'integral': 0.0, 'prev_error': 0.0,
                'zona_frames': 0, 'modo': 'BLOQUEO',
            }
        st = self._st

        if ball is not None:
            bx, by    = ball
            dist_bola = np.hypot(bx - rx, by - ry)

            if en_zona(bx) and dist_bola < 95:
                st['zona_frames'] = min(st['zona_frames'] + 1, 10)
            else:
                st['zona_frames'] = max(st['zona_frames'] - 1, 0)

            modo_anterior = st['modo']

            if st['zona_frames'] >= 2:
                # ── MODO DESPEJE: empuja hacia banda lateral más cercana ───────
                # Posición de ataque: detrás de la pelota (lado del arco propio)
                dx = bx - ox;  dy = by - oy
                mag = np.hypot(dx, dy) + 1e-8
                atk_x = clamp_x(bx + (dx / mag) * 35)
                atk_y = int(np.clip(by + (dy / mag) * 35, goal_y_min, goal_y_max))
                if np.hypot(atk_x - rx, atk_y - ry) < 22:
                    # Ya en posición: empujar hacia el borde de la boca del arco
                    lado_y = goal_y_min if by > h // 2 else goal_y_max
                    target_x = clamp_x(bx)
                    target_y = lado_y
                else:
                    target_x, target_y = atk_x, atk_y
                st['modo'] = 'DESPEJE'
            else:
                # ── MODO BLOQUEO: intercepta trayectoria de la pelota ─────────
                if abs(bx - gx) > 1:
                    t       = (def_x - gx) / float(bx - gx)
                    y_block = gy + t * (by - gy)
                else:
                    y_block = by
                target_x = def_x
                target_y = int(np.clip(y_block, goal_y_min, goal_y_max))
                st['modo'] = 'BLOQUEO'

            if st['modo'] != modo_anterior:
                st['integral']   = 0.0
                st['prev_error'] = 0.0
        else:
            st['zona_frames'] = 0
            st['modo']        = 'BLOQUEO'
            target_x, target_y = def_x, h // 2

        ang = robot['angle']
        ang_to_tgt = (np.degrees(np.arctan2(target_y - ry, target_x - rx)) + 360) % 360
        error_ang  = ((ang_to_tgt - ang + 180) % 360) - 180

        turn_pid, st['integral'], st['prev_error'] = \
            control_pid_angular(error_ang, DT, st['integral'], st['prev_error'])

        turn     = int(np.clip(turn_pid, -90, 90))
        dist_tgt = np.hypot(target_x - rx, target_y - ry)
        return compute_pwm(turn, 1 if dist_tgt > 20 else 0), (int(target_x), int(target_y))


strategy_portero = GoalkeeperStrategy()

# ----------------------------------------------------------
# DELANTERO – COBERTURA DEFENSIVA
# ----------------------------------------------------------
class GameController:
    """Coordinación de roles y generación de la trama serial."""

    @staticmethod
    def strategy_cobertura_delantero(robot, ball, own_goal, opp_goal, w, h,
                               obstaculos, frame_w, frame_h,
                               radio_colision, fuerza_max, borde_seguridad, fuerza_borde):
        """Cubre el tercio propio mientras la pelota está lejos del delantero."""
        if robot is None:
            return "0,0", None

        rx, ry = robot['pos']
        target_x = w // 3 + 15 if own_goal[0] == 0 else 2 * w // 3 - 15
        target_y = int(np.clip(ball[1] if ball is not None else h // 2,
                               h // 5, 4 * h // 5))

        dist_al_tgt = np.hypot(target_x - rx, target_y - ry)
        target_x, target_y = aplicar_repulsion(
            (rx, ry), (target_x, target_y), obstaculos,
            radio_colision, fuerza_max, dist_al_tgt)
        target_x, target_y = aplicar_repulsion_bordes(
            target_x, target_y, rx, ry, frame_w, frame_h, borde_seguridad, fuerza_borde)
        target = keep_out_goal_areas((target_x, target_y), own_goal, opp_goal, w, h)
        return drive_to_point_stable(robot, target, stop_distance=28)


    @staticmethod
    def strategy_delantero_polivalente(robot, ball, own_goal, opp_goal, w, h,
                                  obstaculos, predictors, frame_w, frame_h,
                                  radio_colision, fuerza_max, borde_seguridad,
                                  fuerza_borde, distancia_detras_pelota):
        """Un único DEL cubre en defensa y recupera/ataca cuando puede disputar la pelota."""
        if robot is None or ball is None:
            ROBOT_STATE[robot_DEL] = 'ESPERA'
            return '0,0', None

        bx, _ = ball
        rx, ry = robot['pos']
        distancia = np.hypot(bx - rx, ball[1] - ry)
        tercio_propio = bx < w / 3 if own_goal[0] == 0 else bx > 2 * w / 3
        mitad_propia = bx < w / 2 if own_goal[0] == 0 else bx > w / 2

        # La cobertura solo se mantiene mientras la pelota está lejos: cuando se
        # acerca, DEL sale a recuperarla sin esperar a otro robot de campo.
        if tercio_propio and distancia > 220:
            ROBOT_STATE[robot_DEL] = 'COBERTURA'
            return strategy_cobertura_delantero(
                robot, ball, own_goal, opp_goal, w, h, obstaculos, frame_w, frame_h,
                radio_colision, fuerza_max, borde_seguridad, fuerza_borde)

        modo = 'PRESION' if mitad_propia else 'ATAQUE'
        if ROBOT_STATE.get(robot_DEL) != modo and hasattr(strategy_delantero, '_st'):
            strategy_delantero._st = {'integral': 0.0, 'prev_error': 0.0}
        ROBOT_STATE[robot_DEL] = modo
        return strategy_delantero(
            robot, ball, own_goal, opp_goal, w, h, obstaculos, predictors[robot_DEL],
            frame_w, frame_h, radio_colision, fuerza_max, borde_seguridad, fuerza_borde,
            distancia_detras_pelota)


    @staticmethod
    def calcular_logica(own_robots, ball_pos, opp_goal, own_goal, w, h,
                        obstaculos, predictors, frame_w, frame_h, rep_config):
        """
        Controla POR y DEL; conserva el canal central en cero para el maestro actual.
        """
        comandos     = {role: '0,0' for role in ACTIVE_ROLES}
        debug_targets= {}

        robot_por = own_robots.get(robot_POR)
        if robot_por:
            cmd, tgt = strategy_portero(robot_por, ball_pos, own_goal, opp_goal, w, h)
            comandos[robot_POR] = cmd
            if tgt is not None:
                debug_targets[robot_POR] = tgt

        cmd_del, target_del = strategy_delantero_polivalente(
            own_robots.get(robot_DEL), ball_pos, own_goal, opp_goal, w, h,
            obstaculos, predictors, frame_w, frame_h,
            rep_config['RADIO_COLISION'],
            rep_config['FUERZA_REPULSION_MAX'],
            rep_config['BORDE_DIST_SEGURIDAD'],
            rep_config['FUERZA_REPULSION_BORDE'],
            rep_config['DISTANCIA_DETRAS_PELOTA']
        )
        comandos[robot_DEL] = cmd_del
        if target_del is not None:
            debug_targets[robot_DEL] = target_del

        # Robots no detectados: giro suave de búsqueda
        if ball_pos is not None:
            for role in ACTIVE_ROLES:
                if own_robots.get(role) is None:
                    comandos[role] = search_pwm(role)
                    if role == robot_DEL:
                        ROBOT_STATE[robot_DEL] = 'BUSQUEDA'

        # Calibración per-robot
        for role in ACTIVE_ROLES:
            comandos[role] = apply_robot_calibration(comandos[role], role)

        # El firmware espera POR|MED|DEL: el antiguo MED siempre recibe parada.
        trama = f"<{comandos[robot_POR]}|0,0|{comandos[robot_DEL]}>\n"
        return trama, debug_targets


# Alias públicos para conservar compatibilidad con el código existente.
strategy_cobertura_delantero = GameController.strategy_cobertura_delantero
strategy_delantero_polivalente = GameController.strategy_delantero_polivalente
calcular_logica = GameController.calcular_logica

# ----------------------------------------------------------
# DELANTERO – RECUPERACIÓN / ÓRBITA / APROXIMACIÓN / DISPARO
# ----------------------------------------------------------
class AttackerStrategy:
    """Estrategia con memoria PID propia; invocable como la función original."""

    def __call__(self, robot, ball, own_goal, opp_goal, w, h,
                           obstaculos, predictor_del, frame_w, frame_h,
                           radio_colision, fuerza_max, borde_seguridad, fuerza_borde,
                           distancia_detras_pelota):
        """
        RECUPERACIÓN : gira lentamente para reaparecer ante la cámara.
        ÓRBITA       : sortea la pelota si bloquea el camino al punto de ataque.
        APROXIMACIÓN : se dirige al punto detrás de la pelota.
        DISPARO      : empuja la pelota hacia la portería rival.
        """
        if robot is None or ball is None:
            return "0,0", None

        if predictor_del.is_recovery():
            direccion = 1 if (time.time() * 2) % 2 < 1 else -1
            return compute_pwm(direccion * 20, 0), None

        rx, ry = robot['pos']
        bx, by = ball
        ox, oy = opp_goal
        dx     = bx - ox;  dy = by - oy
        mag    = np.hypot(dx, dy) + 1e-8
        dist_atk = max(35, distancia_detras_pelota)
        atk_x = bx + (dx / mag) * dist_atk
        atk_y = by + (dy / mag) * dist_atk
        dist_al_atk = np.hypot(atk_x - rx, atk_y - ry)

        # ── Detección de trayectoria bloqueada (ÓRBITA) ───────────
        CLEARANCE_ORBIT = 36
        lvx = atk_x - rx;  lvy = atk_y - ry
        ll  = np.hypot(lvx, lvy) + 1e-8
        cross  = abs(lvx * (ry - by) - lvy * (rx - bx)) / ll
        t_proj = ((bx - rx) * lvx + (by - ry) * lvy) / (ll ** 2)
        path_blocked = (cross < CLEARANCE_ORBIT) and (0.05 < t_proj < 0.95)

        if path_blocked and dist_al_atk > 25:
            lnx = lvy / ll;  lny = -lvx / ll
            side = 1 if (rx - bx) * lnx + (ry - by) * lny >= 0 else -1
            target_x = int(np.clip(bx + lnx * side * (CLEARANCE_ORBIT + 8),
                                   borde_seguridad, frame_w - borde_seguridad))
            target_y = int(np.clip(by + lny * side * (CLEARANCE_ORBIT + 8),
                                   borde_seguridad, frame_h - borde_seguridad))
        elif dist_al_atk < 22:
            target_x, target_y = int(bx), int(by)
        else:
            target_x, target_y = int(atk_x), int(atk_y)

        # Impide que el delantero entre al área propia.
        target_x, target_y = keep_out_goal_areas(
            (target_x, target_y), own_goal, opp_goal, w, h)

        dist_al_target = np.hypot(target_x - rx, target_y - ry)
        target_x, target_y = aplicar_repulsion(
            (rx, ry), (target_x, target_y), obstaculos,
            radio_colision, fuerza_max, dist_al_target)
        target_x, target_y = aplicar_repulsion_bordes(
            target_x, target_y, rx, ry, frame_w, frame_h, borde_seguridad, fuerza_borde)

        target_x, target_y = keep_out_goal_areas(
            (target_x, target_y), own_goal, opp_goal, w, h)
        dist_al_target = np.hypot(target_x - rx, target_y - ry)
        ang = robot['angle']
        ang_to_tgt = (np.degrees(np.arctan2(target_y - ry, target_x - rx)) + 360) % 360
        error_ang  = ((ang_to_tgt - ang + 180) % 360) - 180

        if not hasattr(self, '_st'):
            self._st = {'integral': 0.0, 'prev_error': 0.0}
        st = self._st
        turn_pid, st['integral'], st['prev_error'] = \
            control_pid_angular(error_ang, DT, st['integral'], st['prev_error'])

        turn = int(np.clip(turn_pid, -90, 90))
        return compute_pwm(turn, 1 if dist_al_target > 15 else 0), (int(target_x), int(target_y))


strategy_delantero = AttackerStrategy()

# ============================================================
# DELANTERO POLIVALENTE: COBERTURA / PRESIÓN / ATAQUE
# ============================================================


# ============================================================
# VENTANA DE CALIBRACIÓN (Lab polar con trackbars)
# ============================================================
class CalibrationWindow(tk.Toplevel):
    def __init__(self, parent, camera_settings):
        super().__init__(parent)
        self.parent          = parent
        self.camera_settings = camera_settings
        self.title("Calibración de Campo y Colores")
        self.geometry("800x650")
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        try:
            self.camera = open_video_source(camera_settings['index'],
                                            camera_settings['source'], camera_settings['url'])
        except Exception:
            self.destroy()
            raise
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        self._apply_camera_params()

        self.calibration_colors = ['naranja', 'amarillo', 'rosa', 'verde', 'azul', 'rojo']
        script_dir = os.path.dirname(os.path.abspath(__file__))

        # ── Pre-cargar colores ya calibrados ─────────────────────────────
        # Si lab_calibration.json existe, los colores presentes se omiten y
        # el usuario llega directamente al primer color que falta (ej. 'rojo').
        existing_colors = load_json(os.path.join(script_dir, 'lab_calibration.json')) or {}
        self.lab_ranges = {
            c: existing_colors[c]
            for c in self.calibration_colors
            if c in existing_colors
        }

        # ── Pre-cargar esquinas del campo si ya existen ───────────────────
        # Si cancha_config.json existe, saltamos la fase de selección de campo.
        existing_field = load_json(os.path.join(script_dir, 'cancha_config.json'))
        if existing_field and 'puntos_cancha' in existing_field:
            self.points = existing_field['puntos_cancha']
            self.state  = 'calibrate'
        else:
            self.points = []
            self.state  = 'select_field'

        self.raw_frame  = None
        self._adjusting_color = False
        self._lab_precomp = None

        self.current_image = None
        self.disp_offset_x = self.disp_offset_y = 0
        self.disp_w = self.disp_h = 1
        self.raw_img_w = self.raw_img_h = 1

        controls = tk.Frame(self)
        controls.pack(fill=tk.X)
        tk.Button(controls, text='Elegir las 4 esquinas otra vez',
                  command=self.reset_field).pack(side=tk.LEFT, padx=4, pady=4)
        tk.Button(controls, text='Recalibrar los 6 colores',
                  command=self.reset_colors).pack(side=tk.LEFT, padx=4, pady=4)
        self.image_label = tk.Label(self)
        self.image_label.pack(fill=tk.BOTH, expand=True)
        self.image_label.bind("<Button-1>", self.mouse_callback)

        self.info_var = tk.StringVar()
        self._running = True
        self.update_info()
        self.update_frame()
        self.update()

    def reset_field(self):
        if self._adjusting_color:
            return
        self.points = []
        self.state = 'select_field'
        self.update_info()

    def reset_colors(self):
        if self._adjusting_color:
            return
        self.lab_ranges = {}
        self.state = 'calibrate' if len(self.points) == 4 else 'select_field'
        self.update_info()

    def _apply_camera_params(self):
        # Los ajustes de imagen se aplican en software igual que en App.
        pass

    def update_info(self):
        if self.state == 'select_field':
            self.info_var.set("Clic en las 4 esquinas del campo (UL, UR, LR, LL)")
        elif self.state == 'calibrate':
            if len(self.lab_ranges) < len(self.calibration_colors):
                nc = self.calibration_colors[len(self.lab_ranges)]
                self.info_var.set(f"Clic sobre el color: {nc.upper()}")
            else:
                self.info_var.set("Todos los colores calibrados. Cierra la ventana.")
        elif self.state == 'done':
            self.info_var.set("Calibración guardada.")

    def mouse_callback(self, event):
        if self._adjusting_color:
            return
        img_x = event.x - self.disp_offset_x
        img_y = event.y - self.disp_offset_y
        if img_x < 0 or img_x >= self.disp_w or img_y < 0 or img_y >= self.disp_h:
            return
        scale_x = self.raw_img_w / self.disp_w
        scale_y = self.raw_img_h / self.disp_h
        x_real  = int(img_x * scale_x)
        y_real  = int(img_y * scale_y)

        if self.state == 'select_field' and len(self.points) < 4:
            self.points.append((x_real, y_real))
            if len(self.points) == 4:
                script_dir = os.path.dirname(os.path.abspath(__file__))
                save_json(os.path.join(script_dir, 'cancha_config.json'),
                          {'puntos_cancha': self.points})
                self.state = 'calibrate'
                self.update_info()

        elif self.state == 'calibrate' and self.raw_frame is not None:
            if len(self.lab_ranges) >= len(self.calibration_colors):
                return
            lab_frame = cv2.cvtColor(self.raw_frame, cv2.COLOR_BGR2Lab)
            y1 = max(0, y_real-4); y2 = min(lab_frame.shape[0], y_real+5)
            x1 = max(0, x_real-4); x2 = min(lab_frame.shape[1], x_real+5)
            roi       = lab_frame[y1:y2, x1:x2]
            mean_lab  = cv2.mean(roi)
            l_val     = int(mean_lab[0])
            a_val     = float(mean_lab[1]) - 128.0
            b_val     = float(mean_lab[2]) - 128.0
            mag_val   = int(np.sqrt(a_val**2 + b_val**2))
            ang_val   = int((np.degrees(np.arctan2(b_val, a_val)) + 360) % 360)
            L_M, R_M, T_M = 60, 50, 35

            color_name = self.calibration_colors[len(self.lab_ranges)]
            win_name   = f"Lab polar – {color_name}  (L={l_val} R={mag_val} θ={ang_val}°)"
            cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(win_name, 800, 500)
            cv2.createTrackbar('L Min',  win_name, max(l_val-L_M, 0),   255, lambda _: None)
            cv2.createTrackbar('L Max',  win_name, min(l_val+L_M, 255), 255, lambda _: None)
            cv2.createTrackbar('R Min',  win_name, max(mag_val-R_M, 0),   180, lambda _: None)
            cv2.createTrackbar('R Max',  win_name, min(mag_val+R_M, 180), 180, lambda _: None)
            cv2.createTrackbar('Th Min', win_name, max(ang_val-T_M, 0),   360, lambda _: None)
            cv2.createTrackbar('Th Max', win_name, min(ang_val+T_M, 360), 360, lambda _: None)

            l_min = l_max = r_min = r_max = t_min = t_max = None
            self._adjusting_color = True
            while True:
                self.update()
                if not self._running:
                    l_min = None
                    break
                if cv2.getWindowProperty(win_name, cv2.WND_PROP_VISIBLE) < 1:
                    l_min = None
                    break
                l_min = cv2.getTrackbarPos('L Min',  win_name)
                l_max = cv2.getTrackbarPos('L Max',  win_name)
                r_min = cv2.getTrackbarPos('R Min',  win_name)
                r_max = cv2.getTrackbarPos('R Max',  win_name)
                t_min = cv2.getTrackbarPos('Th Min', win_name)
                t_max = cv2.getTrackbarPos('Th Max', win_name)

                mask    = lab_polar_mask(self.raw_frame, l_min, l_max, r_min, r_max,
                                         t_min, t_max, _precomp=self._lab_precomp)
                res     = cv2.bitwise_and(self.raw_frame, self.raw_frame, mask=mask)
                h_win   = 480
                scale   = h_win / self.raw_frame.shape[0]
                w_win   = int(self.raw_frame.shape[1] * scale)
                left_p  = cv2.resize(self.raw_frame, (w_win, h_win))
                right_p = cv2.resize(res, (w_win, h_win))
                combined = np.hstack([left_p, right_p])
                cv2.putText(combined, "g/Esp=guardar  Esc=cancelar",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                cv2.imshow(win_name, combined)

                key = cv2.waitKey(1) & 0xFF
                if key in (ord(' '), ord('g')):
                    break
                elif key == 27:
                    l_min = None
                    break
            try:
                cv2.destroyWindow(win_name)
            except cv2.error:
                pass
            self._adjusting_color = False
            if not self._running:
                return

            if l_min is not None:
                self.lab_ranges[color_name] = {
                    'l_min': l_min, 'l_max': l_max,
                    'r_min': r_min, 'r_max': r_max,
                    't_min': t_min, 't_max': t_max,
                }
            self.update_info()

            if len(self.lab_ranges) == len(self.calibration_colors):
                script_dir = os.path.dirname(os.path.abspath(__file__))
                save_json(os.path.join(script_dir, 'lab_calibration.json'), self.lab_ranges)
                self.state = 'done'
                self.update_info()

    def update_frame(self):
        if not self._running:
            return
        if self.camera.isOpened():
            ret, frame = self.camera.read()
            if ret:
                if frame.ndim == 3 and frame.shape[2] == 4:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                s = self.camera_settings
                frame = adjust_image(frame, s['brightness'], s['contrast'],
                                     s['saturation'], s['exposure'])
                if self.state != 'select_field' and len(self.points) == 4:
                    frame, _, _ = warp_image(frame, self.points)
                    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)
                    ll, la, lb = cv2.split(lab)
                    ll = self.clahe.apply(ll)
                    mag, angle = cv2.cartToPolar(la.astype(np.float32) - 128.0,
                                                 lb.astype(np.float32) - 128.0,
                                                 angleInDegrees=True)
                    self._lab_precomp = (ll, mag, angle)
                    frame = cv2.cvtColor(cv2.merge((ll, la, lb)),
                                         cv2.COLOR_Lab2BGR)
                else:
                    self._lab_precomp = None
                self.raw_frame = frame.copy()
                display        = frame.copy()
                if self.state == 'select_field':
                    for p in self.points:
                        cv2.circle(display, p, 4, (0,255,0), 1)
                        cv2.circle(display, p, 1, (0,255,255), -1)
                if self.state != 'done':
                    msg = self.info_var.get()
                    (tw, th), _ = cv2.getTextSize(msg, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                    cv2.rectangle(display, (10, display.shape[0]-th-25),
                                  (10+tw+10, display.shape[0]-10), (0,0,0), -1)
                    cv2.putText(display, msg, (15, display.shape[0]-15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
                self.current_image = Image.fromarray(rgb)
                self.show_image()
        self.after(30, self.update_frame)

    def show_image(self):
        if self.current_image is None:
            return
        label_w = self.image_label.winfo_width()
        label_h = self.image_label.winfo_height()
        if label_w < 10 or label_h < 10:
            label_w, label_h = 640, 480
        img_w, img_h = self.current_image.size
        ratio = min(label_w / img_w, label_h / img_h)
        new_w = int(img_w * ratio);  new_h = int(img_h * ratio)
        self.disp_w          = new_w;  self.disp_h          = new_h
        self.disp_offset_x   = (label_w - new_w) // 2
        self.disp_offset_y   = (label_h - new_h) // 2
        if self.raw_frame is not None:
            self.raw_img_h, self.raw_img_w = self.raw_frame.shape[:2]
        resized = self.current_image.resize((new_w, new_h), Image.Resampling.LANCZOS)
        tk_img  = ImageTk.PhotoImage(resized)
        self.image_label.config(image=tk_img)
        self.image_label.image = tk_img

    def on_close(self):
        self._running = False
        if self.camera.isOpened():
            self.camera.release()
        self.destroy()

# ============================================================
# APLICACIÓN PRINCIPAL
# ============================================================
class App:
    BRIGHT_DEFAULT     = 128
    CONTRAST_DEFAULT   = 128
    SATURATION_DEFAULT = 128
    WB_DEFAULT         = 4000

    def __init__(self, root):
        self.root  = root
        self.root.title("RoboFútbol – Torneo Zacatecano VSSS  (MGApp)")
        self.root.geometry("1200x800")

        self.camera_index = tk.IntVar(value=0)
        self.port_var     = tk.StringVar(value="")
        self.connected    = False
        self.paused       = False
        self.simulacion   = True
        self.camera       = None
        self._camera_opening      = False
        self._frame_reader_running= False
        self._reader_stop = threading.Event()
        self._reader_thread = None
        self._camera_results = queue.SimpleQueue()
        self._closing = False
        self._latest_frame        = None
        self._last_camera_frame   = None
        self._frame_lock          = threading.Lock()
        self._display_frame       = None
        self._display_lock        = threading.Lock()
        self._processing_active   = False
        self.serial_conn  = None

        self.own_goal_side    = tk.StringVar(value="izquierda")
        self.my_color         = tk.StringVar(value="azul")
        self.distancia_detras = tk.IntVar(value=25)
        self.show_raw         = tk.BooleanVar(value=False)

        self.radio_colision        = tk.IntVar(value=35)
        self.fuerza_repulsion_max  = tk.IntVar(value=80)
        self.borde_dist_seguridad  = tk.IntVar(value=30)
        self.fuerza_repulsion_borde= tk.IntVar(value=90)

        self.auto_cam_var     = tk.BooleanVar(value=False)
        self.cam_source       = tk.StringVar(value="local")
        self.iphone_url_var   = tk.StringVar(value="")
        self.corregir_dist_var= tk.BooleanVar(value=False)

        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.field_config_path = os.path.join(script_dir, 'cancha_config.json')
        self.color_config_path = os.path.join(script_dir, 'lab_calibration.json')
        self.calib_cam_path    = os.path.join(script_dir, 'camera_calib.json')
        self.points     = []
        self.colors_lab = None

        self.camera_matrix = None
        self.dist_coeffs   = None
        self.load_camera_calibration()
        if self.camera_matrix is not None:
            self.corregir_dist_var.set(True)

        self.clahe    = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        self.kalman   = init_kalman()

        self.memoria_robots_own = {
            robot_POR: {'pos': None, 'angle': 0, 'lost_frames': 999},
            robot_DEL: {'pos': None, 'angle': 0, 'lost_frames': 999},
        }
        self.memoria_rivales = []
        self.predictors = {
            robot_POR: RobotPredictor(),
            robot_DEL: RobotPredictor(),
        }
        self.frames_sin_bola = 999
        self.last_ball_pos   = None   # última posición detectada real de la pelota
        self.last_serial_time= time.monotonic()
        self.last_trama      = ""
        self.calibrating     = False

        self.build_gui()
        self.load_field_config()
        self.load_color_config()
        self.open_camera()
        self.update()

    # ── Carga de configuraciones ──────────────────────────────────────────
    def load_camera_calibration(self):
        data = load_json(self.calib_cam_path)
        if data and 'camera_matrix' in data and 'dist_coeffs' in data:
            self.camera_matrix = np.array(data['camera_matrix'], dtype=np.float32)
            self.dist_coeffs   = np.array(data['dist_coeffs'],   dtype=np.float32)
            print("Calibración de cámara cargada.")
        else:
            self.camera_matrix = self.dist_coeffs = None
            print("Sin calibración de cámara.")

    def load_field_config(self):
        data = load_json(self.field_config_path)
        self.points = data['puntos_cancha'] if data and 'puntos_cancha' in data else []

    def load_color_config(self):
        data = load_json(self.color_config_path)
        if data:
            self.colors_lab = {color: rng for color, rng in data.items()}
        else:
            # Rojo en Lab polar: ángulo ≈ 340-20° (envuelve 0°, t_min > t_max)
            self.colors_lab = {
                'naranja':  {'l_min': 50,  'l_max': 255, 'r_min': 30, 'r_max': 180, 't_min': 20,  't_max': 65 },
                'amarillo': {'l_min': 80,  'l_max': 255, 'r_min': 25, 'r_max': 180, 't_min': 60,  't_max': 105},
                'rosa':     {'l_min': 40,  'l_max': 255, 'r_min': 25, 'r_max': 180, 't_min': 295, 't_max': 360},
                'verde':    {'l_min': 30,  'l_max': 220, 'r_min': 20, 'r_max': 180, 't_min': 100, 't_max': 175},
                'azul':     {'l_min': 15,  'l_max': 180, 'r_min': 20, 'r_max': 180, 't_min': 195, 't_max': 270},
                'rojo':     {'l_min': 40,  'l_max': 220, 'r_min': 30, 'r_max': 150, 't_min': 340, 't_max': 20 },
            }

    def reload_configs(self):
        self.load_field_config()
        self.load_color_config()
        self.load_camera_calibration()
        if self.camera_matrix is not None:
            self.corregir_dist_var.set(True)
        else:
            self.corregir_dist_var.set(False)
        self._update_dist_button()

    # ── Scroll de Windows / trackpad de macOS ────────────────────────────
    def _on_control_frame_configure(self, event):
        self.left_canvas.configure(scrollregion=self.left_canvas.bbox("all"))

    def _on_canvas_configure(self, event):
        self.left_canvas.itemconfig(self.canvas_window, width=event.width)

    def _on_mousewheel(self, event):
        if not event.delta:
            return
        delta = (-int(event.delta / 120) if sys.platform == 'win32'
                 else (-1 if event.delta > 0 else 1))
        if delta == 0:
            delta = -1 if event.delta > 0 else 1
        self.left_canvas.yview_scroll(delta, "units")

    def _bind_scroll_recursive(self, widget):
        widget.bind("<MouseWheel>", self._on_mousewheel)
        for child in widget.winfo_children():
            self._bind_scroll_recursive(child)

    # ── Puertos seriales ─────────────────────────────────────────────────
    def _get_serial_ports(self):
        # Incluye COM3/COM10 y adaptadores CH340/CP210x sin depender del nombre.
        return [p.device for p in sorted(list_ports.comports(),
                                        key=lambda p: (len(p.device), p.device))]

    def _refresh_ports(self):
        ports = self._get_serial_ports()
        self.port_combo['values'] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        elif not ports:
            self.port_var.set('')

    # ── Construcción de la GUI ───────────────────────────────────────────
    def build_gui(self):
        left_container = tk.Frame(self.root, width=300)
        left_container.pack(side=tk.LEFT, fill=tk.Y, padx=(5,0), pady=5)
        left_container.pack_propagate(False)

        self.left_canvas = tk.Canvas(left_container, width=300, highlightthickness=0,
                                     bg=self.root.cget('bg'))
        self.left_scrollbar = tk.Scrollbar(left_container, orient=tk.VERTICAL,
                                           command=self.left_canvas.yview)
        self.left_canvas.configure(yscrollcommand=self.left_scrollbar.set)
        self.left_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.left_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.control_frame = tk.Frame(self.left_canvas, bg=self.root.cget('bg'))
        self.canvas_window = self.left_canvas.create_window(
            (0,0), window=self.control_frame, anchor=tk.NW)
        self.control_frame.bind("<Configure>", self._on_control_frame_configure)
        self.left_canvas.bind("<Configure>",   self._on_canvas_configure)
        self.left_canvas.bind("<MouseWheel>",  self._on_mousewheel)
        self.control_frame.bind("<MouseWheel>",self._on_mousewheel)

        cf = self.control_frame
        row = 0

        # Cámara
        tk.Label(cf, text="Cámara:").grid(row=row, column=0, sticky='w')
        self.cam_combo = tk.Spinbox(cf, from_=0, to=20, width=5,
                                    textvariable=self.camera_index,
                                    command=self.on_camera_change)
        self.cam_combo.grid(row=row, column=1, sticky='w')
        self.cam_combo.bind('<Return>', self.on_camera_change);  row += 1

        # Puerto serial
        tk.Label(cf, text="Puerto:").grid(row=row, column=0, sticky='w', pady=(10,0))
        self.port_combo = ttk.Combobox(cf, textvariable=self.port_var,
                                       values=self._get_serial_ports(), width=22)
        self.port_combo.grid(row=row, column=1, sticky='w', pady=(10,0));  row += 1
        if self.port_combo['values']:
            self.port_var.set(self.port_combo['values'][0])
        tk.Button(cf, text="Detectar", command=self._refresh_ports, width=8).grid(
            row=row, column=0, columnspan=2, sticky='w', pady=(2,0));  row += 1
        self.conn_btn = tk.Button(cf, text="Conectar", command=self.toggle_connection, bg="lightgray")
        self.conn_btn.grid(row=row, column=0, columnspan=2, sticky='ew', pady=5);  row += 1

        self.pause_btn = tk.Button(cf, text="Pausa", command=self.toggle_pause, bg="lightgray")
        self.pause_btn.grid(row=row, column=0, columnspan=2, sticky='ew', pady=5);  row += 1

        tk.Button(cf, text="Calibración", command=self.open_calibration).grid(
            row=row, column=0, columnspan=2, sticky='ew', pady=5);  row += 1

        # Auto colores (abre calibrar_colores.py)
        tk.Button(cf, text="Auto colores", command=self.auto_calibrate_colors).grid(
            row=row, column=0, columnspan=2, sticky='ew', pady=4);  row += 1

        # Auto camara
        tk.Button(cf, text="Auto camara", command=self.auto_tune_camera_params).grid(
            row=row, column=0, columnspan=2, sticky='ew', pady=4);  row += 1

        tk.Checkbutton(cf, text="Vista Base (RAW)", variable=self.show_raw).grid(
            row=row, column=0, columnspan=2, sticky='w', pady=(10,0));  row += 1

        # Portería propia
        tk.Label(cf, text="Portería propia:").grid(row=row, column=0, sticky='w', pady=(10,0))
        _gf = tk.Frame(cf, bg=self.root.cget('bg'))
        tk.Radiobutton(_gf, text="Izq", variable=self.own_goal_side, value="izquierda",
                       bg=self.root.cget('bg')).pack(side=tk.LEFT)
        tk.Radiobutton(_gf, text="Der", variable=self.own_goal_side, value="derecha",
                       bg=self.root.cget('bg')).pack(side=tk.LEFT)
        _gf.grid(row=row, column=1, sticky='w', pady=(10,0));  row += 1

        # Color uniforme
        tk.Label(cf, text="Color uniforme:").grid(row=row, column=0, sticky='w', pady=(5,0))
        _cof = tk.Frame(cf, bg=self.root.cget('bg'))
        tk.Radiobutton(_cof, text="Azul",     variable=self.my_color, value="azul",
                       bg=self.root.cget('bg')).pack(side=tk.LEFT)
        tk.Radiobutton(_cof, text="Amarillo", variable=self.my_color, value="amarillo",
                       bg=self.root.cget('bg')).pack(side=tk.LEFT)
        _cof.grid(row=row, column=1, sticky='w', pady=(5,0));  row += 1

        # Distancia detrás pelota
        tk.Label(cf, text="Dist. detrás pelota:").grid(row=row, column=0, sticky='w', pady=(10,0))
        row += 1
        tk.Scale(cf, from_=0, to=100, orient=tk.HORIZONTAL,
                 variable=self.distancia_detras, length=150).grid(
            row=row, column=0, columnspan=2, sticky='w');  row += 1

        # Repulsión robots
        tk.Label(cf, text="Repulsión robots", font=('Arial',9,'bold')).grid(
            row=row, column=0, columnspan=2, sticky='w', pady=(10,0));  row += 1
        tk.Label(cf, text="Radio (px):").grid(row=row, column=0, sticky='w')
        tk.Scale(cf, from_=10, to=100, orient=tk.HORIZONTAL,
                 variable=self.radio_colision, length=150).grid(row=row, column=1, sticky='w');  row += 1
        tk.Label(cf, text="Fuerza:").grid(row=row, column=0, sticky='w')
        tk.Scale(cf, from_=10, to=150, orient=tk.HORIZONTAL,
                 variable=self.fuerza_repulsion_max, length=150).grid(row=row, column=1, sticky='w');  row += 1

        # Repulsión bordes
        tk.Label(cf, text="Repulsión bordes", font=('Arial',9,'bold')).grid(
            row=row, column=0, columnspan=2, sticky='w', pady=(10,0));  row += 1
        tk.Label(cf, text="Margen (px):").grid(row=row, column=0, sticky='w')
        tk.Scale(cf, from_=10, to=80, orient=tk.HORIZONTAL,
                 variable=self.borde_dist_seguridad, length=150).grid(row=row, column=1, sticky='w');  row += 1
        tk.Label(cf, text="Fuerza:").grid(row=row, column=0, sticky='w')
        tk.Scale(cf, from_=10, to=150, orient=tk.HORIZONTAL,
                 variable=self.fuerza_repulsion_borde, length=150).grid(row=row, column=1, sticky='w');  row += 1

        # Parámetros de cámara
        tk.Label(cf, text="Parámetros de cámara", font=('Arial',10,'bold')).grid(
            row=row, column=0, columnspan=2, sticky='w', pady=(15,5));  row += 1

        self.exp_var = tk.DoubleVar(value=-6)
        tk.Label(cf, text="Exposición").grid(row=row, column=0, sticky='w')
        self.exp_slider = tk.Scale(cf, from_=-13, to=-1, resolution=0.1, orient=tk.HORIZONTAL,
                                   variable=self.exp_var, length=150, command=self.on_param_change)
        self.exp_slider.grid(row=row, column=1, sticky='w');  row += 1

        self.bright_var = tk.DoubleVar(value=self.BRIGHT_DEFAULT)
        tk.Label(cf, text="Brillo").grid(row=row, column=0, sticky='w')
        self.bright_slider = tk.Scale(cf, from_=0, to=255, orient=tk.HORIZONTAL,
                                      variable=self.bright_var, length=150, command=self.on_param_change)
        self.bright_slider.grid(row=row, column=1, sticky='w');  row += 1

        self.contrast_var = tk.DoubleVar(value=self.CONTRAST_DEFAULT)
        tk.Label(cf, text="Contraste").grid(row=row, column=0, sticky='w')
        self.contrast_slider = tk.Scale(cf, from_=0, to=255, orient=tk.HORIZONTAL,
                                        variable=self.contrast_var, length=150, command=self.on_param_change)
        self.contrast_slider.grid(row=row, column=1, sticky='w');  row += 1

        self.saturation_var = tk.DoubleVar(value=self.SATURATION_DEFAULT)
        tk.Label(cf, text="Saturación").grid(row=row, column=0, sticky='w')
        self.saturation_slider = tk.Scale(cf, from_=0, to=255, orient=tk.HORIZONTAL,
                                          variable=self.saturation_var, length=150, command=self.on_param_change)
        self.saturation_slider.grid(row=row, column=1, sticky='w');  row += 1

        self.wb_var = tk.DoubleVar(value=self.WB_DEFAULT)
        tk.Label(cf, text="Balance: app iPhone").grid(row=row, column=0, sticky='w')
        self.wb_slider = tk.Scale(cf, from_=2800, to=6500, orient=tk.HORIZONTAL,
                                  variable=self.wb_var, length=150, command=self.on_param_change)
        self.wb_slider.grid(row=row, column=1, sticky='w');  row += 1
        self.wb_slider.config(state=tk.DISABLED)

        tk.Checkbutton(cf, text="Auto (Exp manual)", variable=self.auto_cam_var,
                       command=self.toggle_auto_camera).grid(
            row=row, column=0, columnspan=2, sticky='w', pady=(10,0));  row += 1

        self.dist_btn = tk.Button(cf, text="Corregir distorsión radial",
                                  command=self.toggle_corregir_distorsion, relief=tk.RAISED)
        self.dist_btn.grid(row=row, column=0, columnspan=2, sticky='ew', pady=(8,0));  row += 1
        self._update_dist_button()

        # Fuente de cámara (local / iPhone URL)
        tk.Label(cf, text="Fuente de cámara", font=('Arial',10,'bold')).grid(
            row=row, column=0, columnspan=2, sticky='w', pady=(15,5));  row += 1
        _sf = tk.Frame(cf, bg=self.root.cget('bg'))
        tk.Radiobutton(_sf, text="Local / iPhone USB", variable=self.cam_source, value="local",
                       bg=self.root.cget('bg'), command=self._on_cam_source_change).pack(side=tk.LEFT)
        tk.Radiobutton(_sf, text="iPhone URL",variable=self.cam_source, value="iphone",
                       bg=self.root.cget('bg'), command=self._on_cam_source_change).pack(side=tk.LEFT)
        _sf.grid(row=row, column=0, columnspan=2, sticky='w');  row += 1

        tk.Label(cf, text="URL iPhone:").grid(row=row, column=0, sticky='w', pady=(5,0))
        self.iphone_url_entry = tk.Entry(cf, textvariable=self.iphone_url_var,
                                         width=20, state=tk.DISABLED)
        self.iphone_url_entry.grid(row=row, column=1, sticky='w', pady=(5,0));  row += 1
        self.iphone_connect_btn = tk.Button(cf, text="Conectar cámara iPhone",
                                             command=self.open_camera, state=tk.DISABLED)
        self.iphone_connect_btn.grid(row=row, column=0, columnspan=2, sticky='ew', pady=(5,0));  row += 1
        tk.Label(cf, text="URL de video de la app (HTTP/RTSP).\nUSB: abre DroidCam y elige Local.",
                 font=('Arial',8), fg='gray').grid(row=row, column=0, columnspan=2, sticky='w');  row += 1

        # Área de visualización
        self.display_frame = tk.Frame(self.root, bg='black')
        self.display_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.display_frame.bind("<Configure>", self.on_resize)
        self.image_label = tk.Label(self.display_frame, bg='black')
        self.image_label.pack(fill=tk.BOTH, expand=True)

        self._bind_scroll_recursive(self.control_frame)

    # ── Cámara ────────────────────────────────────────────────────────────
    def open_camera(self):
        if self._camera_opening:
            return
        source = self.cam_source.get()
        url = self.iphone_url_var.get().strip()
        if source == 'iphone':
            try:
                parts = urlsplit(url)
                valid = parts.scheme.lower() in ('http', 'https', 'rtsp', 'rtsps') and parts.hostname
            except ValueError:
                valid = False
            if not valid:
                messagebox.showerror('Cámara iPhone', 'Introduce la URL de video de tu app. '
                                     'Si usas DroidCam por USB, selecciona Local / iPhone USB.')
                return
        self._camera_opening       = True
        self._frame_reader_running = False
        self._reader_stop.set()
        old = self.camera
        self.camera = None
        with self._frame_lock:
            self._latest_frame = None
            self._last_camera_frame = None
        threading.Thread(target=self._open_camera_bg,
                         args=(old, self._reader_thread, self.camera_index.get(), source, url),
                         daemon=True).start()

    def _open_camera_bg(self, old_cam, old_reader, idx, source, url):
        # El trabajador no lee variables Tk ni modifica widgets.
        cam, error = None, None
        try:
            if old_reader is not None:
                old_reader.join()
            if old_cam is not None:
                old_cam.release()
            if not self._closing:
                cam = open_video_source(idx, source, url)
        except Exception as exc:
            error = str(exc)
        if self._closing:
            if cam is not None:
                cam.release()
        else:
            self._camera_results.put((cam, error))

    def _on_camera_ready(self, cam, error):
        self._camera_opening = False
        self.camera = cam
        if error or cam is None:
            messagebox.showerror('Error de cámara', error or 'No se pudo abrir la cámara.')
            return
        self.apply_camera_params()
        self._frame_reader_running = True
        self._reader_stop = threading.Event()
        self._reader_thread = threading.Thread(target=self._frame_reader_loop,
                                                args=(cam, self._reader_stop), daemon=True)
        self._reader_thread.start()

    def _frame_reader_loop(self, cam, stop_event):
        while not stop_event.is_set():
            if cam is not None and cam.isOpened():
                ret, frame = cam.read()
                if ret and frame is not None and not stop_event.is_set():
                    if frame.ndim == 3 and frame.shape[2] == 4:
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                    with self._frame_lock:
                        self._latest_frame = frame
                        self._last_camera_frame = frame
                elif not ret:
                    stop_event.wait(0.03)
            else:
                stop_event.wait(0.03)

    def apply_camera_params(self, event=None):
        # Brillo/contraste/saturación/exposición ya se aplican en adjust_image.
        # Los rangos del driver de Windows no equivalen a los de los sliders.
        # En iPhone, foco y balance de blancos se ajustan en la app de cámara.
        pass

    def toggle_auto_camera(self):
        auto  = self.auto_cam_var.get()
        state = tk.DISABLED if auto else tk.NORMAL
        if auto:
            self.bright_var.set(self.BRIGHT_DEFAULT)
            self.contrast_var.set(self.CONTRAST_DEFAULT)
            self.saturation_var.set(self.SATURATION_DEFAULT)
            self.wb_var.set(self.WB_DEFAULT)
        for sl in [self.bright_slider, self.contrast_slider,
                   self.saturation_slider]:
            sl.config(state=state)
        self.apply_camera_params()

    def auto_tune_camera_params(self):
        """Ajusta automáticamente brillo, saturación y contraste según estadísticas del frame."""
        if self.camera is None or not self.camera.isOpened():
            messagebox.showerror("Error", "La cámara no está activa.")
            return
        with self._frame_lock:
            frame = None if self._last_camera_frame is None else self._last_camera_frame.copy()
        if frame is None:
            messagebox.showerror("Error", "No se pudo leer imagen de la cámara.")
            return
        hsv     = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mean_v  = float(np.mean(hsv[:, :, 2]))
        mean_s  = float(np.mean(hsv[:, :, 1]))
        contrast= float(np.std(gray))

        brightness   = int(np.clip(self.bright_var.get()    + (125 - mean_v) * 0.35, 70, 190))
        saturation   = int(np.clip(self.saturation_var.get()+ (95  - mean_s) * 0.25, 80, 180))
        contrast_val = int(np.clip(self.contrast_var.get()  + (55  - contrast) * 0.30, 80, 180))

        self.auto_cam_var.set(False)
        self.bright_var.set(brightness)
        self.saturation_var.set(saturation)
        self.contrast_var.set(contrast_val)
        for sl in [self.bright_slider, self.contrast_slider, self.saturation_slider]:
            sl.config(state=tk.NORMAL)
        self.apply_camera_params()
        messagebox.showinfo("Auto cámara", "Parámetros ajustados automáticamente.")

    def auto_calibrate_colors(self):
        """Usa la misma cámara local/URL sin abrir un segundo consumidor."""
        self.open_calibration(recalibrate_colors=True)

    def on_camera_change(self, event=None):
        self.open_camera()

    def _on_cam_source_change(self):
        if self.cam_source.get() == "iphone":
            self.iphone_url_entry.config(state=tk.NORMAL)
            self.iphone_connect_btn.config(state=tk.NORMAL)
            self.cam_combo.config(state=tk.DISABLED)
        else:
            self.iphone_url_entry.config(state=tk.DISABLED)
            self.iphone_connect_btn.config(state=tk.DISABLED)
            self.cam_combo.config(state=tk.NORMAL)
            self.open_camera()

    def on_param_change(self, event=None):
        self.apply_camera_params()

    def toggle_corregir_distorsion(self):
        self.corregir_dist_var.set(not self.corregir_dist_var.get())
        self._update_dist_button()

    def _update_dist_button(self):
        if self.corregir_dist_var.get():
            self.dist_btn.config(relief=tk.SUNKEN, text="Corregir distorsión radial (ON)")
        else:
            self.dist_btn.config(relief=tk.RAISED, text="Corregir distorsión radial (OFF)")

    # ── Conexión serial ───────────────────────────────────────────────────
    def toggle_connection(self):
        if self.connected:
            if self.serial_conn:
                self.serial_conn.close()
            self.serial_conn = None
            self.connected   = False
            self.simulacion  = True
            self.conn_btn.config(text="Conectar", bg="lightgray")
        else:
            port = self.port_var.get()
            if not port:
                messagebox.showerror("Error", "Selecciona o detecta un puerto primero.")
                return
            self.conn_btn.config(text="Conectando…", bg="yellow", state=tk.DISABLED)
            threading.Thread(target=self._connect_serial_bg, args=(port,), daemon=True).start()

    def _connect_serial_bg(self, port):
        try:
            conn = serial.Serial(port, SERIAL_BAUDRATE, timeout=SERIAL_TIMEOUT)
            conn.setDTR(False);  conn.setRTS(False);  time.sleep(2)
            self.serial_conn = conn
            self.connected   = True
            self.simulacion  = False
            self.root.after(0, lambda: self.conn_btn.config(
                text="Desconectar", bg="lightgreen", state=tk.NORMAL))
        except Exception as e:
            msg = str(e)
            self.root.after(0, lambda: [
                self.conn_btn.config(text="Conectar", bg="lightgray", state=tk.NORMAL),
                messagebox.showerror("Error", f"No se pudo conectar a {port}:\n{msg}")
            ])

    def toggle_pause(self):
        self.paused = not self.paused
        if self.paused:
            self.pause_btn.config(text="Reanudar", bg="orange")
            if self.connected and self.serial_conn:
                try:
                    self.serial_conn.write("<0,0|0,0|0,0>\n".encode())
                except Exception:
                    pass
        else:
            self.pause_btn.config(text="Pausa", bg="lightgray")

    # ── Calibración ───────────────────────────────────────────────────────
    def open_calibration(self, recalibrate_colors=False):
        if self._camera_opening:
            messagebox.showinfo("Espera", "La cámara se está inicializando.")
            return
        if self.camera is None or not self.camera.isOpened():
            messagebox.showerror("Error", "La cámara no está activa.")
            return
        camera_settings = {
            'index':      self.camera_index.get(),
            'source':     self.cam_source.get(),
            'url':        self.iphone_url_var.get().strip(),
            'brightness': self.bright_var.get(),
            'contrast':   self.contrast_var.get(),
            'saturation': self.saturation_var.get(),
            'exposure':   self.exp_var.get(),
            'wb':         self.wb_var.get(),
            'auto_wb':    self.auto_cam_var.get(),
        }
        self._frame_reader_running = False
        self._reader_stop.set()
        self.calibrating = True
        # Espera al lector antes de liberar el VideoCapture (también en Windows).
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=3)
            if self._reader_thread.is_alive():
                self.calibrating = False
                messagebox.showerror('Cámara ocupada', 'La cámara todavía está terminando '
                                     'una lectura. Intenta de nuevo en unos segundos.')
                return
        old_cam = self.camera
        self.camera = None
        old_cam.release()
        with self._frame_lock:
            self._latest_frame = None
        try:
            cal_win = CalibrationWindow(self.root, camera_settings)
            if recalibrate_colors:
                cal_win.reset_colors()
            self.root.wait_window(cal_win)
        except Exception as exc:
            messagebox.showerror('Calibración', str(exc))
        finally:
            self.calibrating = False
            self.reload_configs()
            self.open_camera()

    # ── Bucle principal (threaded) ────────────────────────────────────────
    def _snapshot_params(self):
        return {
            'corregir_dist':          self.corregir_dist_var.get(),
            'show_raw':               self.show_raw.get(),
            'colors_lab':             self.colors_lab,
            'points':                 list(self.points),
            'own_goal_side':          self.own_goal_side.get(),
            'my_color':               self.my_color.get(),
            'camera_matrix':          self.camera_matrix,
            'dist_coeffs':            self.dist_coeffs,
            'radio_colision':         self.radio_colision.get(),
            'fuerza_repulsion_max':   self.fuerza_repulsion_max.get(),
            'borde_dist_seguridad':   self.borde_dist_seguridad.get(),
            'fuerza_repulsion_borde': self.fuerza_repulsion_borde.get(),
            'distancia_detras':       self.distancia_detras.get(),
            'brightness':             self.bright_var.get(),
            'contrast':               self.contrast_var.get(),
            'saturation':             self.saturation_var.get(),
            'exposure':               self.exp_var.get(),
        }

    def update(self):
        if self._closing:
            return
        try:
            cam, error = self._camera_results.get_nowait()
        except queue.Empty:
            pass
        else:
            self._on_camera_ready(cam, error)
        if self.calibrating:
            self.root.after(30, self.update);  return

        with self._display_lock:
            display_img          = self._display_frame
            self._display_frame  = None
        if display_img is not None:
            self.display_image(display_img)

        if not self._processing_active:
            with self._frame_lock:
                frame              = self._latest_frame
                self._latest_frame = None
            if frame is not None:
                params = self._snapshot_params()
                self._processing_active = True
                threading.Thread(target=self._process_frame_bg,
                                 args=(frame, params), daemon=True).start()
        self.root.after(30, self.update)

    def _process_frame_bg(self, frame, params):
        try:
            display_img = self._compute_display(frame, params)
        except Exception as e:
            import traceback
            print(f"[ERROR] {traceback.format_exc()}")
            display_img = frame.copy()
            cv2.putText(display_img, f"Error: {e}", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
        with self._display_lock:
            self._display_frame = display_img
        self._processing_active = False

    def _compute_display(self, frame, params):
        if params['corregir_dist']:
            frame = undistort_frame(frame, params['camera_matrix'], params['dist_coeffs'])

        frame = adjust_image(frame, params['brightness'], params['contrast'],
                             params['saturation'], params['exposure'])

        if len(params['points']) != 4 or params['colors_lab'] is None:
            display_img = frame.copy()
            cv2.putText(display_img, "Calibración necesaria: haz clic en Calibración",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            return display_img

        warped, w, h = warp_image(frame, params['points'])
        if params['show_raw']:
            display_img = frame.copy()
            self.draw_hud(display_img, None, None, frame.shape[1], frame.shape[0])
            return display_img

        # CLAHE en canal L de Lab
        lab = cv2.cvtColor(warped, cv2.COLOR_BGR2Lab)
        ll, la, lb = cv2.split(lab)
        ll  = self.clahe.apply(ll)
        bgr_enhanced = cv2.cvtColor(cv2.merge((ll, la, lb)), cv2.COLOR_Lab2BGR)
        _blur        = cv2.GaussianBlur(bgr_enhanced, (0, 0), sigmaX=1.2)
        display_img  = cv2.addWeighted(bgr_enhanced, 1.5, _blur, -0.5, 0)

        # Pre-cómputo Lab polar (una sola conversión por frame)
        _a = la.astype(np.float32) - 128.0
        _b = lb.astype(np.float32) - 128.0
        _mag, _angle = cv2.cartToPolar(_a, _b, angleInDegrees=True)
        _precomp = (ll, _mag, _angle)

        # Pelota: la detección usa la última posición REAL (no predicción Kalman)
        # para evitar el bucle de realimentación que desplaza la pelota.
        ball_pos = detect_ball(bgr_enhanced, params['colors_lab']['naranja'],
                               _precomp=_precomp, last_pos=self.last_ball_pos)
        if ball_pos:
            self.kalman.correct(np.array([[ball_pos[0]], [ball_pos[1]]], np.float32))
            self.frames_sin_bola = 0
            self.last_ball_pos   = ball_pos
            target_ball = ball_pos
        else:
            self.frames_sin_bola += 1
            if self.frames_sin_bola < 12:
                # Solo usar predicción Kalman para un puñado de frames perdidos
                pred = self.kalman.predict()
                target_ball = (int(pred[0][0]), int(pred[1][0]))
            else:
                target_ball = None

        # Robots
        own_robots, rivales = identify_team(bgr_enhanced, params['colors_lab'],
                                            params['my_color'], _precomp=_precomp)

        for role in ACTIVE_ROLES:
            det = own_robots.get(role)
            if det is not None:
                self.predictors[role].update(det['pos'])
                self.memoria_robots_own[role]['pos']         = det['pos']
                self.memoria_robots_own[role]['angle']       = det['angle']
                self.memoria_robots_own[role]['lost_frames'] = 0
            else:
                self.predictors[role].update(None)
                self.memoria_robots_own[role]['pos']         = self.predictors[role].filtered_pos
                self.memoria_robots_own[role]['lost_frames'] = self.predictors[role].lost_frames

        self.memoria_rivales = rivales
        self.draw_field_references(display_img, w, h, params['own_goal_side'])

        own_active      = {}
        lista_obstaculos= []
        for role in ACTIVE_ROLES:
            mem = self.memoria_robots_own[role]
            if mem['lost_frames'] < MEMORY_FRAMES and mem['pos'] is not None:
                own_active[role] = {'pos': mem['pos'], 'angle': mem['angle']}
                self.draw_own_robot(display_img, role, mem['pos'], mem['angle'])
                lista_obstaculos.append(mem['pos'])

        for rival in rivales:
            if rival['pos'] is not None:
                lista_obstaculos.append(rival['pos'])
                self.draw_rival_robot(display_img, rival['pos'])

        self.draw_repulsion_zones(display_img, w, h, lista_obstaculos,
                                  params['radio_colision'], params['borde_dist_seguridad'])

        own_goal_side = params['own_goal_side']
        if own_goal_side == 'izquierda':
            own_goal, opp_goal = (0, h//2), (w, h//2)
        else:
            own_goal, opp_goal = (w, h//2), (0, h//2)

        if target_ball is not None:
            rep_config = {
                'RADIO_COLISION':          params['radio_colision'],
                'FUERZA_REPULSION_MAX':    params['fuerza_repulsion_max'],
                'BORDE_DIST_SEGURIDAD':    params['borde_dist_seguridad'],
                'FUERZA_REPULSION_BORDE':  params['fuerza_repulsion_borde'],
                'DISTANCIA_DETRAS_PELOTA': params['distancia_detras'],
            }
            trama, debug_targets = calcular_logica(
                own_active, target_ball, opp_goal, own_goal,
                w, h, lista_obstaculos, self.predictors, w, h, rep_config)
            for tgt in debug_targets.values():
                cv2.circle(display_img, tgt, 8, COLOR_TARGET, 2)
            self.send_serial_if_needed(trama)
            self.draw_hud(display_img, trama, target_ball, w, h)
        else:
            ROBOT_STATE[robot_DEL] = 'ESPERA'
            self.draw_hud(display_img, "<ESPERANDO BOLA>", None, w, h)

        if target_ball:
            cv2.circle(display_img, target_ball, 8, COLOR_BALL, -1)

        return display_img

    # ── Comunicación serial ───────────────────────────────────────────────
    def send_serial_if_needed(self, trama):
        if (self.simulacion or self.paused or not self.connected or self.calibrating
                or self._camera_opening or self._closing):
            return
        now = time.monotonic()
        if (trama != self.last_trama and now - self.last_serial_time > 0.05) or \
           (now - self.last_serial_time > SERIAL_REFRESH_SECONDS):
            try:
                self.serial_conn.write(trama.encode())
                self.last_serial_time = now
                self.last_trama       = trama
            except Exception:
                self.connected  = False
                self.simulacion = True
                self.serial_conn= None
                self.root.after(0, lambda: self.conn_btn.config(text="Conectar", bg="lightgray"))

    # ── Dibujo ────────────────────────────────────────────────────────────
    def draw_repulsion_zones(self, img, w, h, obstaculos, radio=None, borde=None):
        if radio is None: radio = self.radio_colision.get()
        if borde is None: borde = self.borde_dist_seguridad.get()
        for (rx, ry) in obstaculos:
            cv2.circle(img, (rx, ry), radio, COLOR_REPULSION, 1)
        cv2.line(img, (borde, 0),     (borde, h),     COLOR_REPULSION, 1, cv2.LINE_AA)
        cv2.line(img, (w-borde, 0),   (w-borde, h),   COLOR_REPULSION, 1, cv2.LINE_AA)
        cv2.line(img, (0, borde),     (w, borde),     COLOR_REPULSION, 1, cv2.LINE_AA)
        cv2.line(img, (0, h-borde),   (w, h-borde),   COLOR_REPULSION, 1, cv2.LINE_AA)

    def draw_field_references(self, img, w, h, own_goal_side=None):
        if own_goal_side is None:
            own_goal_side = self.own_goal_side.get()
        own = (0, h//2) if own_goal_side == 'izquierda' else (w, h//2)
        opp = (w, h//2) if own_goal_side == 'izquierda' else (0, h//2)
        for pos, color in [(own, (0,255,255)), (opp, (0,0,255))]:
            cv2.circle(img, pos, 20, color, 2)
            cv2.line(img, (pos[0]-20, pos[1]), (pos[0]+20, pos[1]), color, 2)
            cv2.line(img, (pos[0], pos[1]-20), (pos[0], pos[1]+20), color, 2)
        area_w = int(h * GOAL_AREA_HEIGHT_RATIO);  area_d = int(w * GOAL_AREA_DEPTH_RATIO);  gy = h // 2
        ty = gy - area_w // 2;  by = gy + area_w // 2
        if own_goal_side == 'izquierda':
            cv2.rectangle(img, (0, ty), (area_d, by), (255,0,0), 2)
        else:
            cv2.rectangle(img, (w-area_d, ty), (w, by), (255,0,0), 2)

    def draw_own_robot(self, img, role, pos, angle):
        x, y  = pos
        color = {robot_POR: COLOR_POR_OWN,
                 robot_DEL: COLOR_DEL_OWN}.get(role, (0,255,0))
        cv2.circle(img, (x,y), 15, color, 2)
        cv2.putText(img, role, (x-15, y-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
        rad = np.radians(angle)
        dx  = int(15 * np.cos(rad));  dy = int(15 * np.sin(rad))
        cv2.arrowedLine(img, (x,y), (x+dx, y+dy), color, 2, tipLength=0.3)

    def draw_rival_robot(self, img, pos):
        x, y = pos
        cv2.circle(img, (x,y), 15, COLOR_RIVAL, 2)
        cv2.putText(img, "Rival", (x-30, y-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_RIVAL, 2)

    def draw_hud(self, img, trama, ball_pos, frame_w, frame_h):
        overlay = img.copy()
        px, py  = 5, frame_h - 135
        cv2.rectangle(overlay, (px, py), (px+355, frame_h-5), COLOR_HUD_BG, -1)
        cv2.addWeighted(overlay, 0.6, img, 0.4, 0, img)
        y = frame_h - 20
        if self.simulacion:
            txt = "MODO SIMULACION"
        elif self.paused:
            txt = "PAUSA - Comandos detenidos"
        else:
            txt = f"Enviando: {trama.strip() if trama else 'N/A'}"
        cv2.putText(img, txt,              (15, y-80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)
        if ball_pos:
            cv2.putText(img, f"Pelota: X:{ball_pos[0]} Y:{ball_pos[1]}",
                        (15, y-55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
        else:
            cv2.putText(img, "Pelota no encontrada",
                        (15, y-55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
        estado   = ("PAUSA ACTIVA" if self.paused
                    else f"ACTIVO / DEL: {ROBOT_STATE.get(robot_DEL, 'ESPERA')}")
        conn_txt = "Conectado"    if self.connected else "Simulacion"
        cv2.putText(img, f"Estado: {estado}",    (15, y-30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1)
        cv2.putText(img, f"Conexion: {conn_txt}",(15, y-5),  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

    # ── Visualización ─────────────────────────────────────────────────────
    def display_image(self, img):
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.current_image_pil = Image.fromarray(rgb)
        self.show_image()

    def show_image(self):
        if not hasattr(self, 'current_image_pil'):
            return
        label_w = self.image_label.winfo_width()
        label_h = self.image_label.winfo_height()
        if label_w < 10 or label_h < 10:
            label_w, label_h = 640, 480
        img_w, img_h = self.current_image_pil.size
        ratio = min(label_w / img_w, label_h / img_h)
        new_w = int(img_w * ratio);  new_h = int(img_h * ratio)
        resized = self.current_image_pil.resize((new_w, new_h), Image.Resampling.LANCZOS)
        tk_img  = ImageTk.PhotoImage(resized)
        self.image_label.config(image=tk_img)
        self.image_label.image = tk_img

    def on_resize(self, event):
        self.show_image()

    def on_close(self):
        self._closing = True
        self._reader_stop.set()
        self._frame_reader_running  = False
        self._processing_active     = False
        while True:
            try:
                pending_cam, _ = self._camera_results.get_nowait()
            except queue.Empty:
                break
            if pending_cam is not None:
                pending_cam.release()
        time.sleep(0.15)
        if self.serial_conn:
            try:
                self.serial_conn.write("<0,0|0,0|0,0>\n".encode())
                time.sleep(0.1)
                self.serial_conn.close()
            except Exception:
                pass
        cam = self.camera
        reader = self._reader_thread
        def release_camera():
            if reader is not None:
                reader.join()
            if cam is not None:
                cam.release()
        threading.Thread(target=release_camera, daemon=True).start()
        self.root.destroy()

# ============================================================
# PUNTO DE ENTRADA
# ============================================================
if __name__ == "__main__":
    root = tk.Tk()
    app  = App(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
