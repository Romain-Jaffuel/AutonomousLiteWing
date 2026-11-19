#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LiteWing - Test moteur par moteur (diagnostic, aucun vol possible).

Pilote directement m1, m2, m3 puis m4 via motorPowerSet, un seul a la fois :
un seul moteur ne peut pas soulever le drone. Sert a comparer le demarrage
et la vitesse de chaque moteur quand l'un d'eux semble traîner.

Usage :
    python motor_test_per_motor.py                 # 12000, 1.5 s par moteur
    python motor_test_per_motor.py --power 14000
"""
import sys, time, argparse, logging, threading
import cflib.crtp
from cflib.crazyflie import Crazyflie

URI = 'udp://192.168.43.42'
PLAFOND = 60000
TIMEOUT = 10.0
logging.basicConfig(level=logging.ERROR)


class TestUnParUn:
    def __init__(self, uri, power, duree, tous=False):
        self.uri, self.power, self.duree = uri, power, duree
        self.tous = tous
        self.connecte = threading.Event()
        self.cf = Crazyflie(rw_cache='./cache')
        self.cf.connected.add_callback(lambda u: self.connecte.set())

    def _set(self, nom, val):
        self.cf.param.set_value(nom, str(int(val)))
        time.sleep(0.05)

    def _tout_couper(self):
        for m in ('m1', 'm2', 'm3', 'm4'):
            try: self._set(f'motorPowerSet.{m}', 0)
            except Exception: pass
        try: self._set('motorPowerSet.enable', 0)
        except Exception: pass

    def run(self):
        print(f'Ouverture du lien vers {self.uri} ...')
        try:
            self.cf.open_link(self.uri)
        except Exception as e:
            print(f'[ERREUR] open_link : {e}'); return 1

        if not self.connecte.wait(TIMEOUT):
            print('[ERREUR] pas de connexion')
            try: self.cf.close_link()
            except Exception: pass
            return 1
        print('[OK] Connecte. Chargement des parametres...')
        time.sleep(3)

        code = 0
        try:
            self._set('motorPowerSet.enable', 1)
            if self.tous:
                print(f'  --> m1+m2+m3+m4 ensemble a {self.power} '
                      f'pendant {self.duree} s (valeur identique sur les 4)')
                for m in ('m1', 'm2', 'm3', 'm4'):
                    self._set(f'motorPowerSet.{m}', self.power)
                time.sleep(self.duree)
                for m in ('m1', 'm2', 'm3', 'm4'):
                    self._set(f'motorPowerSet.{m}', 0)
                print('      les 4 arretes')
                self._set('motorPowerSet.enable', 0)
                print('Tous les moteurs coupes.')
                try:
                    self.cf.close_link()
                except Exception:
                    pass
                print('Termine.')
                return 0
            for m in ('m1', 'm2', 'm3', 'm4'):
                print(f'  --> {m} a {self.power} pendant {self.duree} s')
                self._set(f'motorPowerSet.{m}', self.power)
                time.sleep(self.duree)
                self._set(f'motorPowerSet.{m}', 0)
                print(f'      {m} arrete')
                time.sleep(1.0)
            self._set('motorPowerSet.enable', 0)
            print('Tous les moteurs coupes.')
        except KeyboardInterrupt:
            print('\n[STOP] interruption -> coupure'); self._tout_couper(); code = 1
        except Exception as e:
            print(f'[ERREUR] {e} -> coupure'); self._tout_couper(); code = 1
        finally:
            try: self.cf.close_link()
            except Exception: pass
        print('Termine.')
        return code


def main():
    p = argparse.ArgumentParser(description='Test moteur par moteur LiteWing')
    p.add_argument('--power', type=int, default=12000, help='0-60000 (defaut 12000)')
    p.add_argument('--duree', type=float, default=1.5, help='secondes par moteur')
    p.add_argument('--tous', action='store_true',
                   help='les 4 moteurs en meme temps, valeur identique')
    p.add_argument('--oui', action='store_true')
    a = p.parse_args()

    if a.power > PLAFOND:
        print(f'[REFUS] power={a.power} depasse le plafond {PLAFOND}.'); return 2
    if a.duree > 5:
        print('[REFUS] duree limitee a 5 s par moteur.'); return 2

    print('=' * 58)
    if a.tous:
        print(' TEST LES 4 MOTEURS ENSEMBLE - poussee identique sur chacun')
    else:
        print(' TEST MOTEUR PAR MOTEUR - un seul moteur a la fois')
    print(f' puissance {a.power} / 65535 , {a.duree} s par moteur')
    if a.tous:
        print(" Regarde si les 4 demarrent et s'arretent ensemble.")
    else:
        print(' Regarde lequel demarre en retard ou tourne moins vite.')
    print('=' * 58)
    if not a.oui:
        for i in (3, 2, 1):
            print(f'  demarrage dans {i} ...'); time.sleep(1)

    cflib.crtp.init_drivers()
    return TestUnParUn(URI, a.power, a.duree, a.tous).run()


if __name__ == '__main__':
    sys.exit(main())
