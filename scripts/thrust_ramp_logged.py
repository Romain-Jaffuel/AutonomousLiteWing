#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Thrust ramp with on-board logging.

Subclasses the ramp in thrust_ramp.py and records what the firmware
actually does: motor.m1..m4, stabilizer.thrust, pm.vbat, roll, pitch.
Writes log_moteurs.csv and log_etat.csv, then prints a summary.

Usage :
    python thrust_ramp_logged.py --debut 33000 --fin 35000 --montee 1.5
"""
import sys
import time
import argparse

import cflib.crtp
from cflib.crazyflie.log import LogConfig

import thrust_ramp as rm

PERIODE_LOG_MS = 50          # 20 Hz
CSV_MOTEURS = 'log_moteurs.csv'
CSV_ETAT = 'log_etat.csv'
PWM_MAX = 65535
SEUIL_SATURATION = 0.98 * PWM_MAX     # on considere sature au-dela
SEUIL_VERDICT_V = 3.7                 # pas de verdict d'equilibrage en dessous


class RampeInstrumentee(rm.RampeMoteurs):
    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.derniere_commande = 0
        self.lignes_moteurs = []     # (t, commande, m1, m2, m3, m4, thrust)
        self.lignes_etat = []        # (t, vbat, roll, pitch)
        self.t_zero = None
        self.t_drone_zero = None
        self.blocs = []

    # on memorise la commande pour pouvoir l'apparier aux mesures
    def _appliquer(self, valeur):
        self.derniere_commande = int(valeur)
        super()._appliquer(valeur)

    # --- mise en place des blocs de log -----------------------------------
    def _horodatage(self, horodatage):
        """Seconds since the first log packet, on the drone's own clock.
        Using the PC clock instead would misalign the two log blocks and
        fold Wi-Fi jitter into every timestamp."""
        if self.t_drone_zero is None:
            self.t_drone_zero = horodatage
        return (horodatage - self.t_drone_zero) / 1000.0

    def _cb_moteurs(self, horodatage, data, logconf):
        if self.t_zero is None:
            return
        self.lignes_moteurs.append((
            self._horodatage(horodatage),
            self.derniere_commande,
            data.get('motor.m1', 0), data.get('motor.m2', 0),
            data.get('motor.m3', 0), data.get('motor.m4', 0),
            data.get('stabilizer.thrust', 0.0),
        ))

    def _cb_etat(self, horodatage, data, logconf):
        if self.t_zero is None:
            return
        self.lignes_etat.append((
            self._horodatage(horodatage),
            data.get('pm.vbat', 0.0),
            data.get('stabilizer.roll', 0.0),
            data.get('stabilizer.pitch', 0.0),
        ))

    def _demarrer_logs(self):
        try:
            a = LogConfig(name='moteurs', period_in_ms=PERIODE_LOG_MS)
            for v in ('motor.m1', 'motor.m2', 'motor.m3', 'motor.m4',
                      'stabilizer.thrust'):
                a.add_variable(v)
            a.data_received_cb.add_callback(self._cb_moteurs)
            self.cf.log.add_config(a)
            a.start()
            self.blocs.append(a)

            b = LogConfig(name='etat', period_in_ms=PERIODE_LOG_MS)
            for v in ('pm.vbat', 'stabilizer.roll', 'stabilizer.pitch'):
                b.add_variable(v)
            b.data_received_cb.add_callback(self._cb_etat)
            self.cf.log.add_config(b)
            b.start()
            self.blocs.append(b)

            self.t_zero = time.time()
            print(f'Enregistrement demarre ({PERIODE_LOG_MS} ms) : '
                  f'motor.m1..m4, stabilizer.thrust, pm.vbat, roll, pitch')
        except Exception as e:
            print(f'[AVERTISSEMENT] log indisponible : {e}')
            print('              la rampe continue, mais sans mesures.')

    def _arreter_logs(self):
        for b in self.blocs:
            try:
                b.stop()
                b.delete()
            except Exception:
                pass

    # Wraps the base-class sequence.
    def _sequence(self):
        # le TOC de log doit etre telecharge avant d'ouvrir un bloc
        time.sleep(3)
        self._demarrer_logs()
        try:
            super()._sequence()
        finally:
            self._arreter_logs()

    # --- sorties -----------------------------------------------------------
    def ecrire_csv(self):
        try:
            with open(CSV_MOTEURS, 'w', encoding='utf-8', newline='') as f:
                f.write('t_s;commande;m1;m2;m3;m4;stabilizer_thrust\n')
                for l in self.lignes_moteurs:
                    f.write('%.3f;%d;%d;%d;%d;%d;%.1f\n' % l)
            with open(CSV_ETAT, 'w', encoding='utf-8', newline='') as f:
                f.write('t_s;vbat;roll;pitch\n')
                for l in self.lignes_etat:
                    f.write('%.3f;%.3f;%.2f;%.2f\n' % l)
            print(f'{len(self.lignes_moteurs)} points -> {CSV_MOTEURS}')
            print(f'{len(self.lignes_etat)} points -> {CSV_ETAT}')
        except Exception as e:
            print(f'[AVERTISSEMENT] ecriture CSV impossible : {e}')

    def _fenetre_vol(self):
        """Echantillons exploitables pour l'analyse.

        On exclut deux zones qui faussaient tous les verdicts :

        (a) le debut de l'enregistrement, ou le firmware n'a pas encore pris
            le premier setpoint : stabilizer.thrust vaut encore 0 alors que la
            commande est deja haute, ce qui donnait un faux ecart de -33000 et
            un faux "hypothese confirmee" ;

        (b) toute la descente, pendant laquelle le drone se pose. Au sol le
            stabilisateur voit une erreur d'assiette qu'il ne peut pas
            corriger, son integrateur s'emballe, un moteur part a 65535 et les
            autres tombent a 0. L'ecart de 65535 qui en resultait ne mesurait
            rien du tout.

        La fenetre va donc du debut de la montee a la fin du palier, reperee
        par le dernier echantillon a la commande maximale.
        """
        pts = [l for l in self.lignes_moteurs if l[1] > 0 and l[6] > 0]
        if len(pts) < 3:
            return []
        cmax = max(l[1] for l in pts)
        dernier = max(i for i, l in enumerate(pts) if l[1] == cmax)
        return pts[:dernier + 1]

    def analyser(self):
        """Verdict automatique, calcule uniquement sur la phase en vol."""
        print('')
        print('=' * 62)
        print(' ANALYSE')
        print('=' * 62)

        if not self.lignes_moteurs:
            print(' Aucune mesure recue : impossible de conclure.')
            print(' (le bloc de log n a pas demarre, voir avertissement plus haut)')
            return

        vol = self._fenetre_vol()
        if not vol:
            print(f' {len(self.lignes_moteurs)} points enregistres, mais aucune')
            print(' phase de montee exploitable. Rien a conclure.')
            return

        t0, t1 = vol[0][0], vol[-1][0]
        print(f' points enregistres       : {len(self.lignes_moteurs)}')
        print(f' points analyses          : {len(vol)}'
              f'   (montee + palier, t={t0:.2f} a {t1:.2f} s)')
        print(f' descente exclue          : les {len(self.lignes_moteurs) - len(vol)}'
              f' points restants (drone au sol, mesures sans sens)')

        cmds = [l[1] for l in vol]
        thr = [l[6] for l in vol]
        pwm_max_par_pt = [max(l[2:6]) for l in vol]

        print('')
        print(f' commande max             : {max(cmds)}')
        print(f' stabilizer.thrust max    : {max(thr):.0f}')
        print(f' PWM moteur max atteint   : {max(pwm_max_par_pt)} / {PWM_MAX}')

        # 1) le firmware applique-t-il la poussee demandee ?
        ecarts = [t - c for c, t in zip(cmds, thr)]
        pire = max(ecarts, key=abs)
        print('')
        print(f' ecart stabilizer.thrust - commande : max {pire:+.0f}')
        if abs(pire) < 500:
            print('   -> le firmware applique la poussee demandee, sans la baisser.')
        else:
            print('   -> la poussee appliquee DIFFERE de la commande.')

        # 2) saturation, en vol seulement
        satures = [l for l in vol if max(l[2:6]) >= SEUIL_SATURATION]
        print('')
        if satures:
            print(f' SATURATION EN VOL : {len(satures)} point(s) avec un moteur'
                  f' >= {SEUIL_SATURATION:.0f}')
            print(f'   premiere a t={satures[0][0]:.2f} s, commande {satures[0][1]}')
            print('   -> le stabilisateur ne peut plus augmenter ce moteur :')
            print('      il perd son autorite de correction.')
        else:
            print(' Aucun moteur sature en vol : marge de correction conservee.')

        # Battery first: the balance verdict depends on it.
        etat = [x for x in self.lignes_etat if t0 <= x[0] <= t1]
        v = [x[1] for x in etat] if etat else []
        v_moy = (sum(v) / len(v)) if v else None

        if v:
            print('')
            print(f' batterie en vol : {v[0]:.2f} V -> min {min(v):.2f} V'
                  f'   (affaissement {v[0] - min(v):.2f} V,'
                  f' moyenne {v_moy:.2f} V)')
            if v[0] - min(v) > 0.25:
                print('   -> affaissement net : la poussee reelle baisse')
                print('      pendant le run a commande constante.')
            if v[0] - min(v) < 0.05 and v[0] < 3.60:
                print('   -> affaissement quasi nul a basse tension : pack')
                print('      epuise, deja a son plancher en charge.')
            if min(v) < 3.30:
                print('   -> sous 3,30 V en charge : menage la cellule.')

        # Motor spread = balance indicator, but only at the steady state and
        # only at comparable voltage. Motor spin-up produces a large transient
        # (one motor starts before the others) that says nothing about balance,
        # so keep the points at max command, or the last quarter of the climb.
        cmax = max(cmds)
        etabli = [l for l in vol if l[1] == cmax]
        if len(etabli) < 5:
            etabli = vol[max(0, int(len(vol) * 0.75)):]

        ecarts_m = sorted(max(l[2:6]) - min(l[2:6]) for l in etabli)
        median = ecarts_m[len(ecarts_m) // 2]
        pic = etabli[-1]
        transitoire = max(max(l[2:6]) - min(l[2:6]) for l in vol)

        print('')
        print(f' dernier point (t={pic[0]:.2f} s, commande {pic[1]}) :')
        print(f'   m1={pic[2]}  m2={pic[3]}  m3={pic[4]}  m4={pic[5]}')
        print(f' ecart entre moteurs au regime etabli ({len(etabli)} points) :')
        print(f'   median {median}, mini {ecarts_m[0]}, maxi {ecarts_m[-1]}'
              + (f'   a {v_moy:.2f} V' if v_moy else ''))
        print(f' (pic au demarrage des moteurs : {transitoire}, transitoire)')

        # The spread grows as voltage drops at constant balance: more PWM is
        # needed for the same corrective torque. Below SEUIL_VERDICT_V the
        # number is not comparable, so no verdict is given.
        if v_moy is None:
            print('   -> aucune tension enregistree : pas de verdict.')
        elif v_moy < SEUIL_VERDICT_V:
            print(f'   -> PAS DE VERDICT : {v_moy:.2f} V, sous'
                  f' {SEUIL_VERDICT_V:.1f} V.')
            print('      A basse tension l ecart grossit a equilibrage')
            print('      constant. Ne comparer qu a tension egale.')
        elif median > 6000:
            print('   -> ecart important : centre de gravite nettement decale.')
        elif median > 3000:
            print('   -> ecart modere : equilibrage ameliorable.')
        else:
            print('   -> ecart faible : equilibrage correct.')
