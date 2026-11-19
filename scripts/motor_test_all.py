#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Spin all four motors at low thrust, no lift.

Capped at THRUST_PLAFOND so the props turn without the drone leaving the
ground. Hold the drone or weigh it down, clear area, away from the face.

Usage :
    python motor_test_all.py                   # 14000 pendant 3 s
    python motor_test_all.py --thrust 12000 --duree 2
"""
import sys
import time
import argparse
import logging
import threading

import cflib.crtp
from cflib.crazyflie import Crazyflie

URI = 'udp://192.168.43.42'

THRUST_MINI = 10001        # seuil bas du firmware
THRUST_PLAFOND = 20000     # refuse above this: props spin, no lift
TIMEOUT_CONNEXION = 10.0
PERIODE = 0.05             # 20 Hz : le firmware coupe si on se tait trop longtemps
RAMPE = 0.6                # secondes de montee et de descente progressives

logging.basicConfig(level=logging.ERROR)


class TestMoteurs:
    def __init__(self, uri, thrust, duree):
        self.uri = uri
        self.thrust = thrust
        self.duree = duree

        self.connecte = threading.Event()
        self.fini = threading.Event()
        self.erreur = None

        self.cf = Crazyflie(rw_cache='./cache')
        self.cf.connected.add_callback(self._on_connected)
        self.cf.connection_failed.add_callback(self._on_failed)
        self.cf.connection_lost.add_callback(self._on_lost)
        self.cf.disconnected.add_callback(self._on_disconnected)

    def _on_connected(self, uri):
        print(f'[OK] Connecte a {uri} (LED bleue LINK)')
        self.connecte.set()

    def _on_failed(self, uri, msg):
        self.erreur = f'Connexion refusee : {msg}'
        self.fini.set()

    def _on_lost(self, uri, msg):
        self.erreur = f'Lien perdu : {msg}'
        self.fini.set()

    def _on_disconnected(self, uri):
        self.fini.set()

    def _envoyer(self, thrust):
        """Un setpoint a plat : pas de roll/pitch/yaw, seulement la poussee."""
        self.cf.commander.send_setpoint(0.0, 0.0, 0.0, int(thrust))

    def _couper(self):
        """Coupe les moteurs, plusieurs fois pour etre sur que ca passe."""
        for _ in range(10):
            try:
                self._envoyer(0)
            except Exception:
                pass
            time.sleep(0.02)
        try:
            self.cf.commander.send_notify_setpoint_stop()
        except Exception:
            pass

    def _sequence(self):
        # Deverrouillage : le firmware exige une poussee nulle avant toute commande.
        print('Deverrouillage (poussee nulle)...')
        for _ in range(10):
            self._envoyer(0)
            time.sleep(0.02)

        try:
            self.cf.platform.send_arming_request(True)
            time.sleep(0.2)
        except Exception:
            pass  # firmware LiteWing : pas forcement d'armement

        print(f'Montee progressive vers {self.thrust} ...')
        t0 = time.time()
        while True:
            e = time.time() - t0
            if e >= RAMPE:
                break
            self._envoyer(THRUST_MINI + (self.thrust - THRUST_MINI) * (e / RAMPE))
            time.sleep(PERIODE)

        print(f'Palier {self.thrust} pendant {self.duree} s ...')
        t0 = time.time()
        while time.time() - t0 < self.duree:
            self._envoyer(self.thrust)
            time.sleep(PERIODE)

        print('Descente progressive...')
        t0 = time.time()
        while True:
            e = time.time() - t0
            if e >= RAMPE:
                break
            self._envoyer(self.thrust - (self.thrust - THRUST_MINI) * (e / RAMPE))
            time.sleep(PERIODE)

        print('Coupure moteurs.')
        self._couper()

    def run(self):
        print(f'Ouverture du lien vers {self.uri} ...')
        try:
            self.cf.open_link(self.uri)
        except Exception as e:
            print(f'[ERREUR] open_link a echoue : {e}')
            return 1

        if not self.connecte.wait(TIMEOUT_CONNEXION):
            print(f'[ERREUR] Pas de connexion apres {TIMEOUT_CONNEXION} s.')
            if self.erreur:
                print(f'         {self.erreur}')
            try:
                self.cf.close_link()
            except Exception:
                pass
            return 1

        code = 0
        try:
            self._sequence()
        except KeyboardInterrupt:
            print('\n[STOP] Interruption clavier -> coupure moteurs.')
            self._couper()
            code = 1
        except Exception as e:
            print(f'[ERREUR] Pendant la sequence : {e} -> coupure moteurs.')
            self._couper()
            code = 1
        finally:
            print('Fermeture du lien...')
            try:
                self.cf.close_link()
            except Exception:
                pass
            self.fini.wait(5.0)

        if self.erreur:
            print(f'[AVERTISSEMENT] {self.erreur}')
        print('Termine.')
        return code


def main():
    p = argparse.ArgumentParser(description='Test helices LiteWing (sans vol)')
    p.add_argument('--thrust', type=int, default=14000,
                   help=f'poussee {THRUST_MINI}-{THRUST_PLAFOND} (defaut 14000)')
    p.add_argument('--duree', type=float, default=3.0,
                   help='duree du palier en secondes (defaut 3)')
    p.add_argument('--oui', action='store_true',
                   help='demarrer sans le compte a rebours')
    a = p.parse_args()

    if a.thrust > THRUST_PLAFOND:
        print(f'[REFUS] thrust={a.thrust} depasse le plafond {THRUST_PLAFOND} '
              f'de ce script (test sans vol).')
        return 2
    if a.thrust < THRUST_MINI:
        print(f'[REFUS] thrust={a.thrust} est sous le seuil {THRUST_MINI}.')
        return 2
    if a.duree > 10:
        print('[REFUS] duree limitee a 10 s.')
        return 2

    print('=' * 58)
    print(' TEST HELICES - le drone doit etre TENU ou LESTE')
    print(f' poussee {a.thrust} / 65535 , palier {a.duree} s , pas de vol')
    print('=' * 58)

    if not a.oui:
        for i in (3, 2, 1):
            print(f'  demarrage dans {i} ...')
            time.sleep(1)

    cflib.crtp.init_drivers()
    return TestMoteurs(URI, a.thrust, a.duree).run()


if __name__ == '__main__':
    sys.exit(main())
