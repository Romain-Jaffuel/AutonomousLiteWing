#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Slow thrust ramp with keyboard abort and threshold logging.

Ramps from --debut to --fin, optional hold, then down to 0 and cuts.
Any key decelerates to CIBLE_DECELERATION and records the thrust reached
in releves_poussee.csv; x cuts after a short reduction; Ctrl+C cuts now.
Safety limits and the physical warning are printed by --help and at start.

Usage :
    python thrust_ramp.py --debut 33000 --fin 35000 --montee 1.5 --palier 1
    python thrust_ramp.py --trim-pitch -0.55 --trim-roll -0.35
"""
import os
import sys
import time
import argparse
import logging
import threading
from datetime import datetime

import cflib.crtp
from cflib.crazyflie import Crazyflie

URI = 'udp://192.168.43.42'

THRUST_MINI = 20000      # floor chosen for this script; the
                         # firmware itself accepts from 10001
PLAFOND = 60000           # garde-fou dur : refus au-dela
TIMEOUT_CONNEXION = 10.0
PAS_SETPOINT = 0.05        # 20 Hz : watchdog du firmware
PAS_DIRECT = 1.00          # ecritures de parametres : tres lentes, mesure
FICHIER_RELEVES = 'releves_poussee.csv'

# Cible de la DECELERATION clavier (appui touche) : on redescend vite a une
# valeur sous la sustentation, puis _couper() finit. La descente normale, elle,
# va jusqu'a 0.
CIBLE_DECELERATION = 20000

# Trim : garde-fou. 1 degre = 0,17 m/s2 de derive laterale. Au-dela de
# TRIM_MAX on refuse, une faute de frappe ferait partir le drone en tonneau.
TRIM_MAX = 10.0

TOUCHES_COUPURE = (b'x', b'X', b'\x1b')   # x ou Echap

# Key x does not cut dead: it sheds BAISSE_X of the current thrust over
# DUREE_BAISSE_X, which damps the drop from a hover, then cuts.
BAISSE_X = 0.2
DUREE_BAISSE_X = 1

logging.basicConfig(level=logging.ERROR)


# --- lecture clavier non bloquante ----------------------------------------
if os.name == 'nt':
    import msvcrt

    def lire_touche():
        """Renvoie la touche pressee (bytes) ou None. Ne bloque jamais.
        Renvoie None sans broncher s'il n'y a pas de console (script lance
        depuis un outil, sortie redirigee...)."""
        try:
            if msvcrt.kbhit():
                t = msvcrt.getch()
                while msvcrt.kbhit():     # vider la file
                    msvcrt.getch()
                return t
        except Exception:
            return None
        return None
else:
    import select
    import termios
    import tty

    _etat_tty = None

    def _mode_direct_tty():
        global _etat_tty
        if sys.stdin.isatty():
            _etat_tty = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

    def _restaurer_tty():
        if _etat_tty is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, _etat_tty)

    def lire_touche():
        if not sys.stdin.isatty():
            return None
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1).encode()
        return None


class ArretDemande(Exception):
    """Levee quand l'utilisateur demande la coupure immediate."""


class RampeMoteurs:
    def __init__(self, uri, debut, fin, montee, descente, palier, direct,
                 trim_roll=0.0, trim_pitch=0.0):
        self.uri = uri
        self.debut = debut
        self.fin = fin
        self.montee = montee
        self.descente = descente
        self.palier = palier
        self.direct = direct
        # Trim : inclinaison constante ajoutee au setpoint, pour annuler une
        # derive systematique due a une reference d'horizontale faussee.
        # En degres. Cote PC uniquement, le drone n'en garde rien.
        self.trim_roll = float(trim_roll)
        self.trim_pitch = float(trim_pitch)
        self.pas = PAS_DIRECT if direct else PAS_SETPOINT

        # La descente va toujours jusqu'a 0 : finir a --debut laissait le drone
        # en vol quand --debut est au-dessus de la sustentation, et la coupure
        # finale le lachait d'un coup.
        # Pente de la descente, en unites par seconde : sert a decelerer au
        # meme rythme meme si on interrompt la montee en route.
        self.pente = max(1.0, fin / descente)

        self.regime_releve = None     # poussee au moment de l'appui
        self.temps_releve = None
        self.poussee_max = debut
        self.evenement = 'rampe complete'

        self.connecte = threading.Event()
        self.fini = threading.Event()
        self.erreur = None

        self.cf = Crazyflie(rw_cache='./cache')
        self.cf.connected.add_callback(self._on_connected)
        self.cf.connection_failed.add_callback(self._on_failed)
        self.cf.connection_lost.add_callback(self._on_lost)
        self.cf.disconnected.add_callback(self._on_disconnected)

    # --- callbacks ---------------------------------------------------------
    def _on_connected(self, uri):
        print(f'[OK] Connecte a {uri}')
        self.connecte.set()

    def _on_failed(self, uri, msg):
        self.erreur = f'Connexion refusee : {msg}'
        self.fini.set()

    def _on_lost(self, uri, msg):
        self.erreur = f'Lien perdu : {msg}'
        self.fini.set()

    def _on_disconnected(self, uri):
        self.fini.set()

    # --- envoi de la poussee ----------------------------------------------
    def _appliquer(self, valeur):
        v = int(max(0, min(PLAFOND, valeur)))
        self.poussee_max = max(self.poussee_max, v)
        if self.direct:
            for m in ('m1', 'm2', 'm3', 'm4'):
                self.cf.param.set_value(f'motorPowerSet.{m}', str(v))
        else:
            self.cf.commander.send_setpoint(self.trim_roll, self.trim_pitch,
                                            0.0, v)

    def _couper(self):
        for _ in range(5):
            try:
                if self.direct:
                    for m in ('m1', 'm2', 'm3', 'm4'):
                        self.cf.param.set_value(f'motorPowerSet.{m}', '0')
                else:
                    self.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
            except Exception:
                pass
            time.sleep(0.03)
        if self.direct:
            try:
                self.cf.param.set_value('motorPowerSet.enable', '0')
            except Exception:
                pass
        else:
            try:
                self.cf.commander.send_notify_setpoint_stop()
            except Exception:
                pass

    # --- clavier -----------------------------------------------------------
    def _baisse_x(self, valeur):
        """Shed BAISSE_X of the current thrust over DUREE_BAISSE_X, then let
        the caller cut.
        Evite la chute seche d'une coupure nette depuis le stationnaire.
        En mode direct les ecritures de parametres sont trop lentes pour une
        rampe : on coupe directement."""
        if self.direct:
            print('  (mode direct : pas de baisse progressive, coupure directe)')
            return
        cible = valeur * (1.0 - BAISSE_X)
        print(f'  Baisse de {BAISSE_X * 100:.0f} % : {int(valeur)} -> {int(cible)}'
              f' en {DUREE_BAISSE_X:.1f} s, puis coupure totale.')
        t0 = time.time()
        while True:
            ecoule = time.time() - t0
            if ecoule >= DUREE_BAISSE_X:
                break
            self._appliquer(valeur + (cible - valeur)
                            * (ecoule / DUREE_BAISSE_X))
            time.sleep(PAS_SETPOINT)
        self._appliquer(cible)
        print(f'  Poussee a {int(cible)}. Coupure.')

    def _verifier_clavier(self, valeur, ecoule):
        """None = continuer. 'decelerer' = appui simple. Leve ArretDemande sur x."""
        t = lire_touche()
        if t is None:
            return None
        if t in TOUCHES_COUPURE:
            self.evenement = f'x : -{BAISSE_X*100:.0f}% en {DUREE_BAISSE_X:.0f}s puis coupure'
            self.regime_releve = int(valeur)
            self.temps_releve = ecoule
            print('')
            print('  ' + '=' * 46)
            print(f'  [X] REGIME NOTE : {int(valeur)} / 65535')
            print(f'      atteint apres {ecoule:.1f} s')
            print('  ' + '=' * 46)
            self._baisse_x(valeur)
            raise ArretDemande()
        self.regime_releve = int(valeur)
        self.temps_releve = ecoule
        self.evenement = 'deceleration clavier'
        print('')
        print('  ' + '=' * 46)
        print(f'  REGIME NOTE : {int(valeur)} / 65535')
        print(f'  atteint apres {ecoule:.1f} s')
        print('  ' + '=' * 46)
        return 'decelerer'

    # --- rampes ------------------------------------------------------------
    def _rampe(self, depart, arrivee, duree, libelle, clavier=True):
        """Interpolation lineaire. Renvoie la poussee atteinte, ou None si
        l'utilisateur a demande la deceleration."""
        print(f'{libelle} : {depart} -> {arrivee} en {duree:.1f} s')
        t0 = time.time()
        dernier = -1.0
        while True:
            ecoule = time.time() - t0
            if ecoule >= duree:
                break
            valeur = depart + (arrivee - depart) * (ecoule / duree)
            self._appliquer(valeur)

            if clavier and self._verifier_clavier(valeur, ecoule) == 'decelerer':
                return None

            if ecoule - dernier >= 1.0:
                print(f'    {ecoule:4.1f} s   poussee {int(valeur)}')
                dernier = ecoule

            reste = self.pas - ((time.time() - t0) - ecoule)
            if reste > 0:
                time.sleep(reste)

        self._appliquer(arrivee)
        print(f'    {duree:4.1f} s   poussee {arrivee}')
        return arrivee

    def _sequence(self):
        if self.direct:
            print('Mode direct (motorPowerSet) : stabilisateur contourne.')
            self.cf.param.set_value('motorPowerSet.enable', '1')
            time.sleep(0.2)
        else:
            print('Mode setpoint : stabilisateur ACTIF, tenir le drone a plat.')
            print('Deverrouillage (poussee nulle)...')
            for _ in range(10):
                self.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
                time.sleep(0.02)
            try:
                self.cf.platform.send_arming_request(True)
                time.sleep(0.2)
            except Exception:
                pass

        print('')
        print(f'>>> Une touche = DECELERATION jusqu a {CIBLE_DECELERATION}'
              f' + releve du regime')
        print(f'>>> x ou Echap = -{BAISSE_X * 100:.0f}% en '
              f'{DUREE_BAISSE_X:.1f}s puis coupure')
        print('')

        # montee
        atteint = self._rampe(self.debut, self.fin, self.montee, 'Montee')

        if atteint is None:
            # appui pendant la montee : on redescend depuis la poussee atteinte,
            # a la meme pente que la descente normale
            depart = self.regime_releve
            duree = max(0.5, (depart - CIBLE_DECELERATION) / self.pente)
            self._rampe(depart, CIBLE_DECELERATION, duree,
                        'Deceleration demandee', clavier=False)
        else:
            # palier eventuel
            if self.palier > 0:
                print(f'Palier {self.fin} pendant {self.palier} s')
                t0 = time.time()
                interrompu = False
                while time.time() - t0 < self.palier:
                    self._appliquer(self.fin)
                    ecoule = self.montee + (time.time() - t0)
                    if self._verifier_clavier(self.fin, ecoule) == 'decelerer':
                        interrompu = True
                        break
                    time.sleep(self.pas)
                if interrompu:
                    self._rampe(self.fin, CIBLE_DECELERATION, self.descente,
                                'Deceleration demandee', clavier=False)
                    print('Coupure moteurs.')
                    self._couper()
                    return

            self._rampe(self.fin, 0, self.descente,
                        'Descente', clavier=False)

        print('Coupure moteurs.')
        self._couper()

    # --- releve ------------------------------------------------------------
    def enregistrer_releve(self):
        nouveau = not os.path.exists(FICHIER_RELEVES)
        try:
            with open(FICHIER_RELEVES, 'a', encoding='utf-8', newline='') as f:
                if nouveau:
                    f.write('horodatage;mode;debut;fin;montee_s;'
                            'regime_releve;temps_s;poussee_max;evenement\n')
                f.write(';'.join([
                    datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                    'direct' if self.direct else 'setpoint',
                    str(self.debut),
                    str(self.fin),
                    f'{self.montee:.1f}',
                    '' if self.regime_releve is None else str(self.regime_releve),
                    '' if self.temps_releve is None else f'{self.temps_releve:.1f}',
                    str(self.poussee_max),
                    # le trim va dans la colonne evenement pour ne pas changer
                    # le nombre de colonnes du CSV existant
                    self.evenement + (
                        f' | trim r={self.trim_roll:+.1f} '
                        f'p={self.trim_pitch:+.1f}'
                        if (self.trim_roll or self.trim_pitch) else ''),
                ]) + '\n')
            print(f'Releve ajoute a {FICHIER_RELEVES}')
        except Exception as e:
            print(f'[AVERTISSEMENT] ecriture du releve impossible : {e}')

    # --- orchestration -----------------------------------------------------
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

        if self.direct:
            print('Chargement des parametres...')
            time.sleep(3)

        code = 0
        try:
            if os.name != 'nt':
                _mode_direct_tty()
            self._sequence()
        except ArretDemande:
            print('')
            print(f'[STOP] Coupure clavier a {self.regime_releve} / 65535.')
            self._couper()
            code = 1
        except KeyboardInterrupt:
            self.evenement = 'Ctrl+C'
            print('\n[STOP] Ctrl+C -> coupure moteurs.')
            self._couper()
            code = 1
        except Exception as e:
            self.evenement = f'erreur: {e}'
            print(f'[ERREUR] Pendant la rampe : {e} -> coupure moteurs.')
            self._couper()
            code = 1
        finally:
            if os.name != 'nt':
                _restaurer_tty()
            print('Fermeture du lien...')
            try:
                self.cf.close_link()
            except Exception:
                pass
            self.fini.wait(5.0)

        print('')
        print(f'Poussee maximale atteinte : {self.poussee_max} / 65535')
        if self.regime_releve is not None:
            print(f'Regime note              : {self.regime_releve} / 65535')
        self.enregistrer_releve()

        if self.erreur:
            print(f'[AVERTISSEMENT] {self.erreur}')
        print('Termine.')
        return code


def main():
    p = argparse.ArgumentParser(
        description='Rampe lente de poussee LiteWing, avec arret clavier')
    p.add_argument('--debut', type=int, default=20000,
                   help='poussee de depart (defaut 20000)')
    p.add_argument('--fin', type=int, default=30000,
                   help=f'poussee haute, max {PLAFOND} (defaut 30000)')
    p.add_argument('--montee', type=float, default=10.0,
                   help='duree de la montee en s (defaut 10)')
    p.add_argument('--descente', type=float, default=10.0,
                   help='duree de la descente en s (defaut 10)')
    p.add_argument('--palier', type=float, default=0.0,
                   help='maintien en haut avant de redescendre (defaut 0)')
    p.add_argument('--trim-roll', type=float, default=0.0, dest='trim_roll',
                   help='inclinaison roll constante en degres, pour annuler '
                        'une derive systematique (defaut 0)')
    p.add_argument('--trim-pitch', type=float, default=0.0, dest='trim_pitch',
                   help='inclinaison pitch constante en degres (defaut 0)')
    p.add_argument('--direct', action='store_true',
                   help='motorPowerSet au lieu de send_setpoint '
                        '(valeurs identiques mais rampe en escalier)')
    p.add_argument('--oui', action='store_true',
                   help='demarrer sans le compte a rebours')
    a = p.parse_args()

    if a.fin > PLAFOND or a.debut > PLAFOND:
        print(f'[REFUS] poussee limitee a {PLAFOND} dans ce script.')
        return 2
    if a.debut < THRUST_MINI or a.fin < THRUST_MINI:
        print(f'[REFUS] poussee sous le seuil firmware {THRUST_MINI}.')
        return 2
    if a.fin < a.debut:
        print('[REFUS] --fin doit etre superieur ou egal a --debut.')
        return 2
    if a.montee <= 0 or a.descente <= 0:
        print('[REFUS] durees de rampe strictement positives.')
        return 2
    if a.montee > 60 or a.descente > 60 or a.palier > 30:
        print('[REFUS] durees trop longues (montee/descente 60 s, palier 30 s).')
        return 2
    if abs(a.trim_roll) > TRIM_MAX or abs(a.trim_pitch) > TRIM_MAX:
        print(f'[REFUS] trim limite a +/- {TRIM_MAX:.0f} degres.')
        return 2
    if a.direct and (a.trim_roll or a.trim_pitch):
        print('[REFUS] le trim n a aucun effet en mode --direct '
              '(motorPowerSet contourne le stabilisateur).')
        return 2

    total = a.montee + a.palier + a.descente

    print('=' * 62)
    print(' RAMPE DE POUSSEE')
    print(f' {a.debut} -> {a.fin} en {a.montee:.0f} s'
          + (f', palier {a.palier:.0f} s' if a.palier > 0 else '')
          + f', descente vers 0 en {a.descente:.0f} s   (total {total:.0f} s)')
    print(f' mode : {"direct / motorPowerSet (escalier)" if a.direct else "send_setpoint, stabilisateur ACTIF"}')
    if a.trim_roll or a.trim_pitch:
        print(f' trim : roll {a.trim_roll:+.1f} deg , pitch {a.trim_pitch:+.1f} deg'
              f'   (derive theorique {0.171 * (a.trim_roll**2 + a.trim_pitch**2)**0.5:.2f} m/s2)')
    else:
        print(' trim : aucun')
    print('')
    print(f' TOUCHE       -> decelere jusqu a {CIBLE_DECELERATION}'
          f' + note le regime, puis coupure')
    print(f' x ou Echap   -> -{BAISSE_X * 100:.0f}% en {DUREE_BAISSE_X:.1f}s puis coupure')
    print(' Ctrl+C       -> coupure immediate (seule sortie instantanee)')
    if a.fin > 20000:
        print('')
        print(' /!\\  Au-dela de 20000 le drone TIRE FORT et peut s echapper.')
        print(' /!\\  Fermement maintenu ou leste, tenu A PLAT, loin du visage.')
    print('=' * 62)

    if not a.oui:
        for i in (5, 4, 3, 2, 1):
            print(f'  demarrage dans {i} ...')
            time.sleep(1)

    cflib.crtp.init_drivers()
    return RampeMoteurs(URI, a.debut, a.fin, a.montee,
                        a.descente, a.palier, a.direct,
                        a.trim_roll, a.trim_pitch).run()


if __name__ == '__main__':
    sys.exit(main())
