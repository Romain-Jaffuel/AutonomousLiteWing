#!/usr/bin/env python3
"""LiteWing command line.

    lw.py dump [--sortie DOSSIER]   firmware parameters and log variables
    lw.py check                     pre-flight checks, no motors
    lw.py ramp ...                  thrust ramp with on-board logging
    lw.py voir [RUN]                summary of a run, reads files only

Only one process may talk to the drone: the UDP driver binds a fixed local
port and the address is hard-coded in the fork. `voir` never connects.
"""
import argparse
import logging
import os
import sys
import threading
import time
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import safety

logging.basicConfig(level=logging.ERROR)

RACINE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOSSIER_DATA = os.path.join(RACINE, 'data')
DOSSIER_RUNS = os.path.join(DOSSIER_DATA, 'runs')


def connecter(attente_toc=4.0):
    """Open the link and wait for the TOCs. cflib is imported lazily so that
    `voir` works without it."""
    import cflib.crtp
    from cflib.crazyflie import Crazyflie

    pret = threading.Event()
    echec = []
    cf = Crazyflie(rw_cache=os.path.join(RACINE, 'cache'))
    cf.connected.add_callback(lambda uri: pret.set())
    cf.connection_failed.add_callback(lambda uri, msg: echec.append(msg))

    cflib.crtp.init_drivers()
    cf.open_link(safety.URI)
    if not pret.wait(safety.TIMEOUT_CONNEXION):
        try:
            cf.close_link()
        except Exception:
            pass
        raise RuntimeError(echec[0] if echec else
                           f'pas de connexion apres {safety.TIMEOUT_CONNEXION} s'
                           ' (relancer une fois, le premier essai echoue parfois)')
    time.sleep(attente_toc)
    return cf


# --- dump -------------------------------------------------------------------
def ecrire_params(cf, chemin):
    """Parameter names with their current values. Values matter: a firmware
    flash changes them, and the diff against this file is the only record."""
    toc = cf.param.toc.toc
    lignes = []
    for groupe in sorted(toc):
        for nom in sorted(toc[groupe]):
            cle = f'{groupe}.{nom}'
            try:
                lignes.append(f'{cle} = {cf.param.get_value(cle)}')
            except Exception as e:
                lignes.append(f'{cle} = <illisible: {type(e).__name__}>')
    with open(chemin, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(lignes) + '\n')
    return len(lignes), len(toc)


def ecrire_log_vars(cf, chemin):
    """Log variable names. They are streams, not state, so no values."""
    toc = cf.log.toc.toc
    lignes = [f'{g}.{n}' for g in sorted(toc) for n in sorted(toc[g])]
    with open(chemin, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(lignes) + '\n')
    return len(lignes), len(toc)


def cmd_dump(a):
    sortie = a.sortie or os.path.join(DOSSIER_DATA, 'baseline')
    os.makedirs(sortie, exist_ok=True)
    print(f'Connexion a {safety.URI} ...')
    cf = connecter()
    try:
        print('[OK] connecte, lecture des TOC')
        p = os.path.join(sortie, 'params.txt')
        n_p, g_p = ecrire_params(cf, p)
        print(f'  {n_p} parametres ({g_p} groupes)      -> {p}')
        l = os.path.join(sortie, 'log_vars.txt')
        n_l, g_l = ecrire_log_vars(cf, l)
        print(f'  {n_l} variables de log ({g_l} groupes) -> {l}')
        with open(os.path.join(sortie, 'meta.txt'), 'w',
                  encoding='utf-8', newline='\n') as f:
            f.write(f'date       : {datetime.now().isoformat(timespec="seconds")}\n')
            f.write(f'uri        : {safety.URI}\n')
            f.write(f'parametres : {n_p} dans {g_p} groupes\n')
            f.write(f'log        : {n_l} dans {g_l} groupes\n')
    finally:
        try:
            cf.close_link()
        except Exception:
            pass
    print('Termine. Apres le flash : lw.py dump --sortie data/apres-flash, puis diff.')
    return 0


# --- ramp -------------------------------------------------------------------
class Arret(Exception):
    """Operator asked for an immediate cut."""


# Log blocks. A CRTP block holds about 26 bytes, hence the split. Motors are
# sampled fastest because they carry the balance information; state and flow
# move slowly enough at half the rate.
BLOCS = {
    'moteurs': (50, ['motor.m1', 'motor.m2', 'motor.m3', 'motor.m4',
                     'stabilizer.thrust']),
    'etat': (100, ['pm.vbat', 'stabilizer.roll', 'stabilizer.pitch',
                   'range.zrange']),
    'flux': (100, ['motion.deltaX', 'motion.deltaY', 'motion.squal',
                   'stateEstimate.x', 'stateEstimate.y']),
}


class Rampe:
    """Linear thrust ramp over send_setpoint, with keyboard abort and logging.

    send_setpoint is used rather than the official library because here the
    thrust number is the measured quantity, so safety.PLAFOND applies to it.
    The library flies by altitude and never passes a thrust value.
    """

    def __init__(self, cf, debut, fin, montee, descente, palier,
                 trim_roll=0.0, trim_pitch=0.0):
        self.cf = cf
        self.debut, self.fin = debut, fin
        self.montee, self.descente, self.palier = montee, descente, palier
        self.trim_roll, self.trim_pitch = trim_roll, trim_pitch
        # Descent always reaches 0. Stopping at --debut left the drone flying
        # whenever --debut was above hover, and the final cut dropped it.
        self.pente = max(1.0, fin / descente)

        self.regime_releve = None
        self.temps_releve = None
        self.poussee_max = debut
        self.evenement = 'rampe complete'
        self.commande = 0

        self.lignes = {nom: [] for nom in BLOCS}
        self.blocs = []
        self.t_zero = None

    # --- control ---
    def _appliquer(self, valeur):
        v = safety.borner(valeur)
        self.commande = v
        self.poussee_max = max(self.poussee_max, v)
        self.cf.commander.send_setpoint(self.trim_roll, self.trim_pitch, 0.0, v)

    def _couper(self):
        for _ in range(5):
            try:
                self.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
            except Exception:
                pass
            time.sleep(0.03)
        try:
            self.cf.commander.send_notify_setpoint_stop()
        except Exception:
            pass

    def _baisse_x(self, valeur):
        """Shed BAISSE_X over DUREE_BAISSE_X, then let the caller cut. A dead
        cut from a hover drops the drone."""
        cible = valeur * (1.0 - safety.BAISSE_X)
        print(f'  baisse {safety.BAISSE_X * 100:.0f} % : {int(valeur)}'
              f' -> {int(cible)} en {safety.DUREE_BAISSE_X:.1f} s, puis coupure')
        t0 = time.time()
        while True:
            e = time.time() - t0
            if e >= safety.DUREE_BAISSE_X:
                break
            self._appliquer(valeur + (cible - valeur) * (e / safety.DUREE_BAISSE_X))
            time.sleep(safety.PAS_SETPOINT)
        self._appliquer(cible)

    def _clavier(self, valeur, ecoule):
        t = safety.lire_touche()
        if t is None:
            return None
        if self.regime_releve is None:      # first press wins
            self.regime_releve = int(valeur)
            self.temps_releve = ecoule
        if t in safety.TOUCHES_COUPURE:
            self.evenement = f'x : -{safety.BAISSE_X * 100:.0f}% puis coupure'
            print(f'\n  [X] regime note {int(valeur)} a {ecoule:.1f} s')
            self._baisse_x(valeur)
            raise Arret()
        self.evenement = 'deceleration clavier'
        print(f'\n  regime note : {int(valeur)} / 65535 a {ecoule:.1f} s')
        return 'decelerer'

    def _rampe(self, depart, arrivee, duree, libelle, clavier=True):
        print(f'{libelle} : {depart} -> {arrivee} en {duree:.1f} s')
        t0, dernier = time.time(), -1.0
        while True:
            ecoule = time.time() - t0
            if ecoule >= duree:
                break
            valeur = depart + (arrivee - depart) * (ecoule / duree)
            self._appliquer(valeur)
            if clavier and self._clavier(valeur, ecoule) == 'decelerer':
                return None
            if ecoule - dernier >= 1.0:
                print(f'  {ecoule:5.1f} s   {int(valeur)}')
                dernier = ecoule
            reste = safety.PAS_SETPOINT - ((time.time() - t0) - ecoule)
            if reste > 0:
                time.sleep(reste)
        self._appliquer(arrivee)
        return arrivee

    # --- logging ---
    def _horodatage(self, ms):
        """Seconds since the first packet, on the drone's clock. The PC clock
        would misalign the blocks and fold Wi-Fi jitter into every sample."""
        if self.t_zero is None:
            self.t_zero = ms
        return (ms - self.t_zero) / 1000.0

    def _logs_demarrer(self):
        from cflib.crazyflie.log import LogConfig
        for nom, (periode, variables) in BLOCS.items():
            try:
                bloc = LogConfig(name=nom, period_in_ms=periode)
                for v in variables:
                    bloc.add_variable(v)
                bloc.data_received_cb.add_callback(
                    lambda ms, data, lc, nom=nom, variables=variables:
                    self.lignes[nom].append(
                        [self._horodatage(ms), self.commande]
                        + [data.get(v, 0) for v in variables]))
                self.cf.log.add_config(bloc)
                bloc.start()
                self.blocs.append(bloc)
            except Exception as e:
                print(f'[AVERTISSEMENT] bloc {nom} indisponible : {e}')
        print(f'Enregistrement : {len(self.blocs)}/{len(BLOCS)} blocs')

    def _logs_arreter(self):
        for b in self.blocs:
            try:
                b.stop()
                b.delete()
            except Exception:
                pass

    # --- sequence ---
    def executer(self):
        print('Deverrouillage (poussee nulle)')
        for _ in range(10):
            self.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
            time.sleep(0.02)
        try:
            self.cf.platform.send_arming_request(True)
            time.sleep(0.2)
        except Exception:
            pass

        self._logs_demarrer()
        print(f'\n  touche = decelere vers {safety.CIBLE_DECELERATION} + note'
              f'\n  x = baisse puis coupure, Ctrl+C = coupure immediate\n')
        try:
            atteint = self._rampe(self.debut, self.fin, self.montee, 'Montee')
            if atteint is None:
                depart = self.regime_releve
                duree = max(0.5, (depart - safety.CIBLE_DECELERATION) / self.pente)
                self._rampe(depart, safety.CIBLE_DECELERATION, duree,
                            'Deceleration', clavier=False)
            else:
                if self.palier > 0:
                    print(f'Palier {self.fin} pendant {self.palier:.1f} s')
                    t0 = time.time()
                    while time.time() - t0 < self.palier:
                        self._appliquer(self.fin)
                        if self._clavier(self.fin,
                                         self.montee + time.time() - t0) == 'decelerer':
                            break
                        time.sleep(safety.PAS_SETPOINT)
                self._rampe(self.fin, 0, self.descente, 'Descente', clavier=False)
            print('Coupure moteurs.')
        finally:
            self._couper()
            self._logs_arreter()

    def ecrire_run(self):
        """One folder per run: the CSVs used to be overwritten, so only what
        was copied by hand survived."""
        dossier = os.path.join(DOSSIER_RUNS,
                               datetime.now().strftime('%Y-%m-%d_%H%M%S'))
        os.makedirs(dossier, exist_ok=True)
        for nom, (_, variables) in BLOCS.items():
            lignes = self.lignes[nom]
            if not lignes:
                continue
            chemin = os.path.join(dossier, f'{nom}.csv')
            entete = ['t_s', 'commande'] + [v.split('.')[-1] for v in variables]
            with open(chemin, 'w', encoding='utf-8', newline='\n') as f:
                f.write(';'.join(entete) + '\n')
                for l in lignes:
                    f.write(';'.join(f'{x:.3f}' if isinstance(x, float) else str(x)
                                     for x in l) + '\n')
        with open(os.path.join(dossier, 'run.txt'), 'w',
                  encoding='utf-8', newline='\n') as f:
            f.write(f'date      = {datetime.now().isoformat(timespec="seconds")}\n')
            f.write(f'debut     = {self.debut}\nfin       = {self.fin}\n')
            f.write(f'montee_s  = {self.montee}\npalier_s  = {self.palier}\n')
            f.write(f'descente_s= {self.descente}\n')
            f.write(f'trim_roll = {self.trim_roll}\ntrim_pitch= {self.trim_pitch}\n')
            f.write(f'poussee_max = {self.poussee_max}\n')
            f.write(f'regime_releve = {self.regime_releve}\n')
            f.write(f'temps_releve  = {self.temps_releve}\n')
            f.write(f'evenement = {self.evenement}\n')
        return dossier


def cmd_ramp(a):
    safety.verifier_poussee(a.debut, a.fin)
    safety.verifier_trim(a.trim_roll, a.trim_pitch)
    safety.verifier_duree(a.montee, 60, 'montee')
    safety.verifier_duree(a.descente, 60, 'descente')
    if a.palier:
        safety.verifier_duree(a.palier, safety.VOL_MAX_S, 'palier')
    if a.fin < a.debut:
        raise safety.Refus('--fin doit etre superieur ou egal a --debut')

    print(f'{a.debut} -> {a.fin} en {a.montee:.1f} s, palier {a.palier:.1f} s,'
          f' descente vers 0 en {a.descente:.1f} s')
    if a.trim_roll or a.trim_pitch:
        print(f'trim roll {a.trim_roll:+.2f} deg, pitch {a.trim_pitch:+.2f} deg')
    if a.fin > 25000:
        print('/!\\ au-dela de 25000 le drone peut quitter le sol :'
              ' tenu, leste ou attache')
    if not a.oui:
        for i in (5, 4, 3, 2, 1):
            print(f'  demarrage dans {i}')
            time.sleep(1)

    cf = connecter(attente_toc=3.0)
    r = Rampe(cf, a.debut, a.fin, a.montee, a.descente, a.palier,
              a.trim_roll, a.trim_pitch)
    code = 0
    try:
        r.executer()
    except Arret:
        print('[STOP] coupure clavier')
        code = 1
    except KeyboardInterrupt:
        r.evenement = 'Ctrl+C'
        print('\n[STOP] Ctrl+C')
        r._couper()
        code = 1
    finally:
        try:
            cf.close_link()
        except Exception:
            pass

    dossier = r.ecrire_run()
    print(f'\nRun -> {dossier}')
    import analyse
    analyse.resumer(dossier)
    return code


# --- check ------------------------------------------------------------------
# One block, 13 bytes: vbat 4 + zrange 4 + deltaX 2 + deltaY 2 + squal 1.
CHECK_VARS = ['pm.vbat', 'range.zrange', 'motion.deltaX', 'motion.deltaY',
              'motion.squal']


def cmd_check(a):
    """Pre-flight checks, no motors. Exit 0 if everything passes."""
    from cflib.crazyflie.log import LogConfig

    echantillons = []
    cf = connecter(attente_toc=3.0)
    try:
        bloc = LogConfig(name='check', period_in_ms=50)
        for v in CHECK_VARS:
            bloc.add_variable(v)
        bloc.data_received_cb.add_callback(
            lambda ms, data, lc: echantillons.append(
                (time.time(), {v: data.get(v, 0) for v in CHECK_VARS})))
        cf.log.add_config(bloc)
        bloc.start()

        time.sleep(1.5)
        n_repos = len(echantillons)
        print(f'>>> Bouge le drone a la main pendant {a.duree:.0f} s, maintenant.')
        time.sleep(a.duree)
        bloc.stop()
        bloc.delete()
    finally:
        try:
            cf.close_link()
        except Exception:
            pass

    if not echantillons:
        print('FAIL  lien      : aucune donnee de log recue')
        return 1
    bouge = [e[1] for e in echantillons[n_repos:]]
    tous = [e[1] for e in echantillons]

    resultats = []
    resultats.append(('PASS', 'lien', f'{len(tous)} echantillons recus'))

    v = [e['pm.vbat'] for e in tous if e['pm.vbat'] > 0]
    if not v:
        resultats.append(('FAIL', 'batterie', 'pas de lecture'))
    else:
        vmin = min(v)
        etat = ('PASS' if vmin >= safety.VBAT_COMPARABLE else
                'FAIL' if vmin < safety.VBAT_PLANCHER else 'WARN')
        resultats.append((etat, 'batterie', f'{vmin:.2f} V'
                          f' (comparable >= {safety.VBAT_COMPARABLE},'
                          f' plancher {safety.VBAT_PLANCHER})'))

    h = [e['range.zrange'] for e in tous if e['range.zrange'] > 0]
    resultats.append(('PASS', 'hauteur', f'{min(h):.0f} a {max(h):.0f} mm')
                     if h else
                     ('FAIL', 'hauteur', 'zrange toujours nul, module absent ?'))

    deplacement = sum(abs(e['motion.deltaX']) + abs(e['motion.deltaY'])
                      for e in bouge)
    resultats.append(('PASS', 'flux', f'{deplacement:.0f} unites pendant le mouvement')
                     if deplacement > 0 else
                     ('FAIL', 'flux', 'aucune reaction au mouvement'))

    sq = sorted(e['motion.squal'] for e in tous)
    med = sq[len(sq) // 2] if sq else 0
    resultats.append(('PASS', 'squal', f'median {med:.0f}') if med > 0 else
                     ('FAIL', 'squal', 'nul, surface trop lisse ou module absent'))

    for etat, nom, detail in resultats:
        print(f'{etat:4}  {nom:9}: {detail}')
    echecs = [r for r in resultats if r[0] == 'FAIL']
    print('FAIL' if echecs else 'PASS', '- ',
          f'{len(echecs)} verification(s) en echec' if echecs
          else 'pret pour un vol')
    return 1 if echecs else 0


# --- voir -------------------------------------------------------------------
def cmd_voir(a):
    import analyse
    dossier = a.run
    if not dossier:
        if not os.path.isdir(DOSSIER_RUNS):
            print(f'[ERREUR] aucun run dans {DOSSIER_RUNS}')
            return 1
        runs = sorted(d for d in os.listdir(DOSSIER_RUNS)
                      if os.path.isdir(os.path.join(DOSSIER_RUNS, d)))
        if not runs:
            print(f'[ERREUR] aucun run dans {DOSSIER_RUNS}')
            return 1
        dossier = os.path.join(DOSSIER_RUNS, runs[-1])
    if not os.path.isdir(dossier):
        print(f'[ERREUR] {dossier} introuvable')
        return 1
    analyse.resumer(dossier)
    if a.png:
        analyse.planche(dossier)
    return 0


def main():
    p = argparse.ArgumentParser(prog='lw.py',
                                description='Mesure et pilotage du drone LiteWing')
    sous = p.add_subparsers(dest='cmd', required=True)

    d = sous.add_parser('dump', help='parametres et variables de log du firmware')
    d.add_argument('--sortie', help='dossier de sortie (defaut data/baseline)')
    d.set_defaults(fn=cmd_dump)

    r = sous.add_parser('ramp', help='rampe de poussee avec enregistrement')
    r.add_argument('--debut', type=int, default=safety.THRUST_MINI)
    r.add_argument('--fin', type=int, default=25000)
    r.add_argument('--montee', type=float, default=5.0)
    r.add_argument('--descente', type=float, default=4.0)
    r.add_argument('--palier', type=float, default=0.0)
    r.add_argument('--trim-roll', type=float, default=0.0, dest='trim_roll')
    r.add_argument('--trim-pitch', type=float, default=0.0, dest='trim_pitch')
    r.add_argument('--oui', action='store_true', help='sans compte a rebours')
    r.set_defaults(fn=cmd_ramp)

    c = sous.add_parser('check', help='verifications avant vol, sans moteur')
    c.add_argument('--duree', type=float, default=3.0,
                   help='duree du mouvement a la main (defaut 3 s)')
    c.set_defaults(fn=cmd_check)

    v = sous.add_parser('voir', help='resume d un run, sans connexion au drone')
    v.add_argument('run', nargs='?', help='dossier de run (defaut : le dernier)')
    v.add_argument('--png', action='store_true', help='produire la planche')
    v.set_defaults(fn=cmd_voir)

    a = p.parse_args()
    try:
        return a.fn(a)
    except safety.Refus as e:
        print(f'[REFUS] {e}')
        return 2
    except RuntimeError as e:
        print(f'[ERREUR] {e}')
        return 1


if __name__ == '__main__':
    sys.exit(main())
