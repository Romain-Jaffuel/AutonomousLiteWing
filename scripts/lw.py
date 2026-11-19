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
                 trim_roll=0.0, trim_pitch=0.0, note=''):
        self.cf = cf
        self.debut, self.fin = debut, fin
        self.montee, self.descente, self.palier = montee, descente, palier
        self.trim_roll, self.trim_pitch = trim_roll, trim_pitch
        self.note = note
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
              a.trim_roll, a.trim_pitch, a.note)
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


# --- diff -------------------------------------------------------------------
# What a firmware flash is most likely to silently reset. The web flasher
# documents neither its version nor whether it keeps these.
CRITIQUES = ('trim', 'flightmode.', 'stabilizer.', 'powerDist.', 'motion.',
             'range.', 'kalman')


def _lire_params(chemin):
    d = {}
    if os.path.exists(chemin):
        for l in open(chemin, encoding='utf-8'):
            if '=' in l:
                k, v = l.split('=', 1)
                d[k.strip()] = v.strip()
    return d


def _lire_noms(chemin):
    if not os.path.exists(chemin):
        return set()
    return {l.strip() for l in open(chemin, encoding='utf-8') if l.strip()}


def cmd_diff(a):
    """Compare two dump folders. Short on purpose: the point is to spot what a
    flash moved, not to read 183 lines."""
    av, ap = a.avant, a.apres
    p_av, p_ap = (_lire_params(os.path.join(d, 'params.txt')) for d in (av, ap))
    if not p_av or not p_ap:
        print(f'[ERREUR] params.txt manquant dans {av if not p_av else ap}')
        return 1

    ajoutes = sorted(set(p_ap) - set(p_av))
    retires = sorted(set(p_av) - set(p_ap))
    changes = sorted(k for k in set(p_av) & set(p_ap) if p_av[k] != p_ap[k])
    l_av, l_ap = (_lire_noms(os.path.join(d, 'log_vars.txt')) for d in (av, ap))

    print(f'{os.path.basename(av)} -> {os.path.basename(ap)}')
    print(f'  parametres : {len(p_av)} -> {len(p_ap)}'
          f'  (+{len(ajoutes)} -{len(retires)} ~{len(changes)})')
    print(f'  log        : {len(l_av)} -> {len(l_ap)}'
          f'  (+{len(l_ap - l_av)} -{len(l_av - l_ap)})')

    for nom, s in (('ajoute', ajoutes), ('retire', retires)):
        if s:
            print(f'  param {nom} : {", ".join(s[:6])}'
                  + (f' ... (+{len(s) - 6})' if len(s) > 6 else ''))

    critiques = [k for k in changes if any(c in k for c in CRITIQUES)]
    autres = [k for k in changes if k not in critiques]
    for k in critiques:
        print(f'  ! {k} : {p_av[k]} -> {p_ap[k]}')
    for k in autres[:5]:
        print(f'    {k} : {p_av[k]} -> {p_ap[k]}')
    if len(autres) > 5:
        print(f'    ... et {len(autres) - 5} autres')
    for nom, s in (('ajoute', set(l_ap) - set(l_av)), ('retire', set(l_av) - set(l_ap))):
        if s:
            print(f'  log {nom} : {", ".join(sorted(s)[:6])}'
                  + (' ...' if len(s) > 6 else ''))
    if not changes and l_av == l_ap:
        print('  identique')
    print(f'  URI inchangee : {safety.URI} (codee en dur dans le driver,'
          ' pas un parametre)')
    return 0


# --- check ------------------------------------------------------------------
# Two blocks: a CRTP block holds about 26 bytes, and these would not fit in one.
CHECK_BLOCS = {
    'capteurs': ['pm.vbat', 'range.zrange', 'motion.deltaX', 'motion.deltaY',
                 'motion.squal'],
    'estime': ['stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z'],
}


def _collecte(cf, duree, sacs):
    fin = time.time() + duree
    while time.time() < fin:
        time.sleep(0.05)
    return {nom: list(v) for nom, v in sacs.items()}


def cmd_check(a):
    """Pre-flight checks, no motors. Exit 0 if everything passes.

    Criteria are reaction-based, not magnitude thresholds: the brief asks for
    thresholds to be calibrated on the first real logs rather than guessed.
    """
    from cflib.crazyflie.log import LogConfig

    sacs = {nom: [] for nom in CHECK_BLOCS}
    blocs, manquants = [], []
    cf = connecter(attente_toc=3.0)
    try:
        for nom, variables in CHECK_BLOCS.items():
            try:
                b = LogConfig(name=nom, period_in_ms=50)
                for v in variables:
                    b.add_variable(v)
                b.data_received_cb.add_callback(
                    lambda ms, data, lc, nom=nom, variables=variables:
                    sacs[nom].append({v: data.get(v, 0) for v in variables}))
                cf.log.add_config(b)
                b.start()
                blocs.append(b)
            except Exception as e:
                manquants.append(f'{nom} ({type(e).__name__})')

        repos = _collecte(cf, 1.5, sacs)
        print(f'>>> Leve et deplace le drone a la main {a.duree:.0f} s,'
              ' au-dessus d une surface texturee et eclairee.')
        _collecte(cf, a.duree, sacs)
    finally:
        for b in blocs:
            try:
                b.stop()
                b.delete()
            except Exception:
                pass
        try:
            cf.close_link()
        except Exception:
            pass

    cap, est = sacs['capteurs'], sacs['estime']
    if not cap:
        print('FAIL  lien     : aucune donnee de log recue')
        return 1
    # everything after the rest phase is the hand-movement window
    bouge = cap[len(repos['capteurs']):]
    r = [('PASS', 'lien', f'{len(cap)} echantillons')]
    if manquants:
        r.append(('FAIL', 'blocs', 'non demarres : ' + ', '.join(manquants)))

    v = [e['pm.vbat'] for e in cap if e['pm.vbat'] > 0]
    if not v:
        r.append(('FAIL', 'batterie', 'pas de lecture'))
    else:
        etat = ('PASS' if min(v) >= safety.VBAT_COMPARABLE else
                'FAIL' if min(v) < safety.VBAT_PLANCHER else 'WARN')
        r.append((etat, 'batterie', f'{min(v):.2f} V (comparable >= '
                  f'{safety.VBAT_COMPARABLE}, plancher {safety.VBAT_PLANCHER})'))

    # Height must not merely exist, it must move when the drone is lifted.
    h = [e['range.zrange'] for e in cap if e['range.zrange'] > 0]
    if not h:
        r.append(('FAIL', 'hauteur', 'zrange nul, module absent ?'))
    elif max(h) - min(h) <= 0:
        r.append(('FAIL', 'hauteur', f'fige a {h[0]:.0f} mm, ne suit pas le mouvement'))
    else:
        r.append(('PASS', 'hauteur', f'{min(h):.0f} a {max(h):.0f} mm'))

    d = sum(abs(e['motion.deltaX']) + abs(e['motion.deltaY']) for e in bouge)
    r.append(('PASS', 'flux', f'{d:.0f} unites pendant le mouvement') if d > 0
             else ('FAIL', 'flux', 'aucune reaction au mouvement'))

    sq = sorted(e['motion.squal'] for e in cap)
    med = sq[len(sq) // 2] if sq else 0
    r.append(('PASS', 'squal', f'median {med:.0f}') if med > 0
             else ('FAIL', 'squal', 'nul, surface lisse ou module absent'))

    if not est:
        r.append(('FAIL', 'estime', 'stateEstimate muet'))
    else:
        bouge_est = est[len(repos['estime']):] or est
        amp = max(max(abs(e[k]) for k in e) for e in bouge_est)
        r.append(('PASS', 'estime', f'repond, amplitude {amp:.2f} m') if amp > 0
                 else ('FAIL', 'estime', 'reste a zero'))

    for etat, nom, detail in r:
        print(f'{etat:4}  {nom:9}: {detail}')
    echecs = [x for x in r if x[0] == 'FAIL']
    print(('FAIL - ' + f'{len(echecs)} verification(s) en echec') if echecs
          else 'PASS - pret pour un vol')
    if a.baseline:
        print('')
        return cmd_dump(argparse.Namespace(sortie=a.baseline))
    return 1 if echecs else 0


# --- vol --------------------------------------------------------------------
def cmd_vol(a):
    """Short supervised hover through the official library.

    NOT YET FLOWN. The library is blocking: takeoff() and hover() do not
    return until they are done, so the limits it does not enforce are policed
    from this thread while the flight runs in another, and any violation calls
    emergency_stop().

    A thrust ceiling is deliberately absent: the library flies with
    send_hover_setpoint, which carries an altitude and no thrust, so there is
    no thrust value to clamp. Height, duration and voltage are what remain.
    """
    safety.verifier_trim(a.trim_roll, a.trim_pitch)
    safety.verifier_duree(a.duree, safety.VOL_MAX_S, 'duree')
    if a.hauteur <= 0 or a.hauteur > safety.HAUTEUR_MAX_M:
        raise safety.Refus(f'hauteur {a.hauteur} hors de '
                           f']0, {safety.HAUTEUR_MAX_M}] m')

    if not a.sans_check:
        print('Pre-vol...')
        if cmd_check(argparse.Namespace(duree=3.0, baseline=None)) != 0:
            print('[STOP] check en echec, vol annule (--sans-check pour forcer)')
            return 1
        print('')

    from litewing import LiteWing

    print(f'Vol : {a.hauteur:.2f} m pendant {a.duree:.1f} s,'
          f' trim roll {a.trim_roll:+.2f} pitch {a.trim_pitch:+.2f}')
    print('/!\\ zone degagee. Ctrl+C coupe, x coupe, toute touche fait atterrir.')
    if not a.oui:
        for i in (5, 4, 3, 2, 1):
            print(f'  demarrage dans {i}')
            time.sleep(1)

    etat = {'motif': None, 'fini': False}

    def voler(drone):
        try:
            drone.takeoff(height=a.hauteur)
            drone.hover(a.duree)
            drone.land()
        except Exception as e:
            etat['motif'] = etat['motif'] or f'erreur de vol : {e}'
        finally:
            etat['fini'] = True

    code = 0
    with LiteWing(ip=safety.IP) as drone:
        drone.connect()
        drone.hover_trim_roll = a.trim_roll
        drone.hover_trim_pitch = a.trim_pitch
        drone.arm()
        fil = threading.Thread(target=voler, args=(drone,), daemon=True)
        debut = time.time()
        fil.start()
        try:
            while not etat['fini']:
                time.sleep(0.05)
                s = drone.read_sensors()
                v = getattr(s, 'battery', 0) or 0
                h = getattr(s, 'height', 0) or 0
                if v and v < safety.VBAT_PLANCHER:
                    etat['motif'] = f'batterie {v:.2f} V sous le plancher'
                elif h > safety.HAUTEUR_MAX_M:
                    etat['motif'] = f'hauteur {h:.2f} m au-dessus du plafond'
                elif time.time() - debut > a.duree + safety.MARGE_VOL_S:
                    etat['motif'] = 'duree depassee'
                elif safety.lire_touche() is not None:
                    etat['motif'] = 'touche'
                if etat['motif']:
                    break
        except KeyboardInterrupt:
            etat['motif'] = 'Ctrl+C'
        if etat['motif']:
            print(f"\n[STOP] {etat['motif']} -> coupure")
            try:
                drone.emergency_stop()
            except Exception:
                pass
            code = 1
        fil.join(timeout=3.0)
    print('Termine.')
    return code


# --- cam --------------------------------------------------------------------
def cmd_cam(a):
    """Measure the camera stream and what it costs the drone's link.

    The comparison is the whole point: the same ping, once idle and once with
    the stream running. One Wi-Fi adapter is enough only if the second stays
    close to the first.
    """
    import camera

    hote = a.url.split('//')[-1].split('/')[0].split(':')[0]
    r = [('PASS', 'camera', f'{hote} joignable')
         if camera.joignable(hote, a.port) else
         ('FAIL', 'camera', f'{hote}:{a.port} injoignable')]

    print(f'Reference : ping {safety.IP} sans flux...')
    env, rec, mn, moy, mx = camera.ping(safety.IP, a.pings)
    ref = moy
    r.append(('PASS', 'ping seul', f'{rec}/{env} recus, {mn}-{mx} ms,'
              f' moyenne {moy:.1f} ms') if rec else
             ('FAIL', 'ping seul', 'drone injoignable'))

    print(f'Flux {a.duree:.0f} s + ping simultane...')
    res = {}
    fil = threading.Thread(
        target=lambda: res.update(camera.lire_flux(a.url, a.duree, a.sortie)),
        daemon=True)
    fil.start()
    env2, rec2, mn2, moy2, mx2 = camera.ping(safety.IP, a.pings)
    fil.join(timeout=a.duree + 10)

    f = res.get('frames', 0)
    r.append(('PASS', 'flux', f"{f} images, {res.get('fps', 0)} fps,"
              f" {res.get('debit_kbs', 0)} ko/s") if f else
             ('FAIL', 'flux', f"aucune image ({res.get('erreur', 'pas de donnees')})"))
    if res.get('ecart_max') is not None:
        r.append(('PASS' if res['ecart_max'] < 1.0 else 'WARN', 'regularite',
                  f"plus long creux {res['ecart_max']:.2f} s"))
    if res.get('reconnexions'):
        r.append(('WARN', 'stabilite', f"{res['reconnexions']} reconnexion(s)"))

    if rec2 and ref:
        delta = moy2 - ref
        r.append(('PASS' if delta < 20 and rec2 == env2 else 'FAIL',
                  'ping+flux', f'{rec2}/{env2} recus, moyenne {moy2:.1f} ms'
                  f' ({delta:+.1f} ms vs sans flux)'))
    else:
        r.append(('FAIL', 'ping+flux', f'{rec2}/{env2} recus'))

    for etat, nom, detail in r:
        print(f'{etat:4}  {nom:10}: {detail}')
    echecs = [x for x in r if x[0] == 'FAIL']
    print(('FAIL - ' + f'{len(echecs)} en echec') if echecs
          else 'PASS - une seule carte Wi-Fi suffit')
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
    r.add_argument('--note', default='', help='champ libre repris dans summary.json')
    r.add_argument('--oui', action='store_true', help='sans compte a rebours')
    r.set_defaults(fn=cmd_ramp)

    o = sous.add_parser('vol', help='vol stationnaire court par la bibliotheque')
    o.add_argument('--hauteur', type=float, default=0.5, help='metres (defaut 0.5)')
    o.add_argument('--duree', type=float, default=3.0, help='secondes (defaut 3)')
    o.add_argument('--trim-roll', type=float, default=0.0, dest='trim_roll')
    o.add_argument('--trim-pitch', type=float, default=0.0, dest='trim_pitch')
    o.add_argument('--note', default='', help='champ libre repris dans summary.json')
    o.add_argument('--sans-check', action='store_true', dest='sans_check')
    o.add_argument('--oui', action='store_true', help='sans compte a rebours')
    o.set_defaults(fn=cmd_vol)

    f = sous.add_parser('diff', help='compare deux dumps, avant et apres flash')
    f.add_argument('avant', nargs='?', default=os.path.join(DOSSIER_DATA, 'baseline'))
    f.add_argument('apres', nargs='?',
                   default=os.path.join(DOSSIER_DATA, 'apres-flash'))
    f.set_defaults(fn=cmd_diff)

    c = sous.add_parser('check', help='verifications avant vol, sans moteur')
    c.add_argument('--duree', type=float, default=3.0,
                   help='duree du mouvement a la main (defaut 3 s)')
    c.add_argument('--baseline', nargs='?', const=os.path.join(DOSSIER_DATA,
                   'baseline'), help='enchainer un dump dans ce dossier')
    c.set_defaults(fn=cmd_check)

    m = sous.add_parser('cam', help='flux camera et cout sur le lien du drone')
    m.add_argument('--url', default='http://192.168.43.50:81/stream')
    m.add_argument('--port', type=int, default=81)
    m.add_argument('--duree', type=float, default=15.0, help='secondes de flux')
    m.add_argument('--pings', type=int, default=10)
    m.add_argument('--sortie', help='dossier ou ecrire quelques images')
    m.set_defaults(fn=cmd_cam)

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
