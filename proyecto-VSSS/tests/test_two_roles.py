"""Cobertura, presión, ataque y compatibilidad serial del equipo POR + DEL."""
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import MGApp as app


class TwoRoleTests(unittest.TestCase):
    def setUp(self):
        app.ROBOT_STATE[app.robot_DEL] = 'ATAQUE'
        self.robot = {'pos': (500, 325), 'angle': 180}
        self.predictors = {role: app.RobotPredictor() for role in app.ACTIVE_ROLES}
        self.config = dict(RADIO_COLISION=35, FUERZA_REPULSION_MAX=80,
                           BORDE_DIST_SEGURIDAD=30, FUERZA_REPULSION_BORDE=90,
                           DISTANCIA_DETRAS_PELOTA=35)

    def strategy(self, ball, own=(0, 325), opp=(750, 325), robot=None):
        return app.strategy_delantero_polivalente(
            robot or self.robot, ball, own, opp, 750, 650, [], self.predictors,
            750, 650, 35, 80, 30, 90, 35)

    def test_only_two_roles_and_delantero_keeps_its_markers(self):
        self.assertEqual(app.ACTIVE_ROLES, ('POR', 'DEL'))
        for uniform in ('azul', 'amarillo'):
            pairs = app.get_role_color_pairs(uniform)
            self.assertEqual(set(pairs.values()), {'POR', 'DEL'})
            self.assertEqual(pairs[('rosa', 'verde')], 'DEL')
            self.assertNotIn(('rojo', 'verde'), pairs)

    def test_old_midfielder_markers_do_not_identify_an_active_robot(self):
        rects = {
            'azul': [((100, 100), (40, 40), 0)],
            'rojo': [((110, 105), (6, 6), 0)],
            'verde': [((110, 95), (6, 6), 0)],
        }
        colors = {name: name for name in ('azul', 'amarillo', 'rosa', 'verde', 'rojo')}
        with patch.object(app, 'get_rects',
                          side_effect=lambda img, color, *a, **k: rects.get(color, [])):
            own, _ = app.identify_team(None, colors, 'azul')
        self.assertEqual(own, {'POR': None, 'DEL': None})

    def test_far_defensive_ball_selects_coverage_left_goal(self):
        _, target = self.strategy((100, 240))
        self.assertEqual(app.ROBOT_STATE['DEL'], 'COBERTURA')
        self.assertEqual(target, (265, 240))

    def test_far_defensive_ball_selects_coverage_right_goal(self):
        robot = {'pos': (150, 325), 'angle': 0}
        _, target = self.strategy((650, 400), own=(750, 325), opp=(0, 325), robot=robot)
        self.assertEqual(app.ROBOT_STATE['DEL'], 'COBERTURA')
        self.assertEqual(target, (485, 400))

    def test_near_defensive_ball_selects_pressure(self):
        robot = {'pos': (260, 325), 'angle': 180}
        with patch.object(app, 'strategy_delantero', return_value=('10,20', (180, 325))) as attack:
            command, _ = self.strategy((200, 325), robot=robot)
        self.assertEqual(command, '10,20')
        self.assertEqual(app.ROBOT_STATE['DEL'], 'PRESION')
        attack.assert_called_once()

    def test_rival_half_selects_attack_even_when_ball_is_far(self):
        with patch.object(app, 'strategy_delantero', return_value=('20,30', (650, 325))):
            command, _ = self.strategy((700, 325))
        self.assertEqual(command, '20,30')
        self.assertEqual(app.ROBOT_STATE['DEL'], 'ATAQUE')

    def test_missing_ball_stops_delantero(self):
        self.assertEqual(self.strategy(None), ('0,0', None))
        self.assertEqual(app.ROBOT_STATE['DEL'], 'ESPERA')

    def test_serial_preserves_slots_and_always_stops_old_med(self):
        own = {'POR': self.robot, 'DEL': self.robot, 'MED': self.robot}
        with patch.object(app, 'strategy_portero', return_value=('10,20', (70, 325))), \
                patch.object(app, 'strategy_delantero_polivalente',
                             return_value=('30,40', (400, 325))):
            frame, targets = app.calcular_logica(
                own, (400, 325), (750, 325), (0, 325), 750, 650, [],
                self.predictors, 750, 650, self.config)
        self.assertEqual(frame, '<10,20|0,0|30,40>\n')
        self.assertEqual(set(targets), {'POR', 'DEL'})

    def test_search_for_missing_robots_does_not_move_old_med(self):
        frame, _ = app.calcular_logica(
            {}, (400, 325), (750, 325), (0, 325), 750, 650, [],
            self.predictors, 750, 650, self.config)
        self.assertEqual(frame.strip()[1:-1].split('|')[1], '0,0')
        self.assertNotEqual(frame.strip()[1:-1].split('|')[2], '0,0')

    def test_coverage_target_stays_outside_own_goal_area(self):
        _, target = self.strategy((20, 325))
        self.assertGreater(target[0], 750 * app.GOAL_AREA_DEPTH_RATIO)


if __name__ == '__main__':
    unittest.main()
