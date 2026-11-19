"""Read a run folder, write summary.json, print a summary, draw the board.

Never connects to the drone. Only one process may hold the UDP link, so the
analysis side reads files.
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import safety

PWM_MAX = 65535
SEUIL_SATURATION = 0.98 * PWM_MAX


def lire_csv(chemin):
    """Rows as dicts of floats, keyed by the header. Returns [] if absent."""
    if not os.path.exists(chemin):
        return []
    with open(chemin, encoding='utf-8') as f:
        lignes = [l.strip() for l in f if l.strip()]
    if len(lignes) < 2:
        return []
    cles = lignes[0].split(';')
    out = []
    for l in lignes[1:]:
        ch = l.split(';')
        if len(ch) != len(cles):
            continue
        try:
            out.append({k: float(v) for k, v in zip(cles, ch)})
        except ValueError:
            continue
    return out


def lire_run_txt(dossier):
    meta = {}
    chemin = os.path.join(dossier, 'run.txt')
    if os.path.exists(chemin):
        for l in open(chemin, encoding='utf-8'):
            if '=' in l:
                k, v = l.split('=', 1)
                meta[k.strip()] = v.strip()
    return meta


def fenetre_vol(moteurs, mode='ramp'):
    """The samples worth analysing.

    For a ramp, climb and hold only. Two zones are excluded because they
    produced wrong verdicts before: the start, where the firmware has not
    taken the first setpoint yet and thrust reads 0 against a high command;
    and the descent, where the drone is on the ground and the stabilizer
    fights the floor, its integrator winding up until one motor sits at 65535
    and the others at 0, which measures nothing.

    For a flight, `commande` is a target height, not a thrust, so the climb
    heuristic does not apply. The window is simply where the firmware is
    driving the motors.
    """
    if mode == 'vol':
        return [r for r in moteurs if r.get('thrust', 0) > 0]
    pts = [r for r in moteurs if r['commande'] > 0 and r.get('thrust', 0) > 0]
    if len(pts) < 3:
        return []
    cmax = max(r['commande'] for r in pts)
    dernier = max(i for i, r in enumerate(pts) if r['commande'] == cmax)
    return pts[:dernier + 1]


def _mediane(valeurs):
    v = sorted(valeurs)
    return v[len(v) // 2] if v else None


def resume(dossier):
    """Build the summary dict. Stable keys, small enough to read directly."""
    moteurs = lire_csv(os.path.join(dossier, 'moteurs.csv'))
    etat = lire_csv(os.path.join(dossier, 'etat.csv'))
    flux = lire_csv(os.path.join(dossier, 'flux.csv'))
    meta = lire_run_txt(dossier)

    mode = meta.get('mode', 'ramp')
    s = {'dossier': os.path.basename(dossier), 'mode': mode,
         'note': meta.get('note', ''), 'meta': meta,
         'points': {'moteurs': len(moteurs), 'etat': len(etat), 'flux': len(flux)},
         'drapeaux': []}

    vol = fenetre_vol(moteurs, mode)
    s['points']['analyses'] = len(vol)
    if vol:
        s['duree_s'] = round(vol[-1]['t_s'] - vol[0]['t_s'], 2)
        s['commande_max'] = int(max(r['commande'] for r in vol))
        s['thrust_max'] = round(max(r['thrust'] for r in vol), 1)
        # The firmware should apply the commanded collective as-is.
        s['ecart_collectif'] = round(
            max((r['thrust'] - r['commande'] for r in vol), key=abs), 1)
        pwm = [max(r[f'm{i}'] for i in (1, 2, 3, 4)) for r in vol]
        s['pwm_max'] = int(max(pwm))
        if s['pwm_max'] >= SEUIL_SATURATION:
            s['drapeaux'].append('saturation')

    # Battery first: the balance verdict depends on it.
    if etat:
        t0 = vol[0]['t_s'] if vol else etat[0]['t_s']
        t1 = vol[-1]['t_s'] if vol else etat[-1]['t_s']
        v = [r['vbat'] for r in etat if t0 <= r['t_s'] <= t1] or \
            [r['vbat'] for r in etat]
        s['vbat_depart'] = round(v[0], 3)
        s['vbat_min'] = round(min(v), 3)
        s['vbat_chute'] = round(v[0] - min(v), 3)
        s['vbat_moyenne'] = round(sum(v) / len(v), 3)
        if s['vbat_depart'] < safety.VBAT_COMPARABLE:
            s['drapeaux'].append('pack_faible')
        if s['vbat_chute'] < 0.05 and s['vbat_depart'] < 3.60:
            s['drapeaux'].append('pack_epuise')
        if s['vbat_min'] < safety.VBAT_PLANCHER:
            s['drapeaux'].append('sous_plancher')
        h = [r['zrange'] for r in etat if r.get('zrange', 0) > 0]
        if h:
            s['hauteur_mm_min'] = round(min(h), 1)
            s['hauteur_mm_max'] = round(max(h), 1)

    # Motor spread, steady state only. Motor spin-up produces a large
    # transient that says nothing about balance.
    if vol:
        cmax = max(r['commande'] for r in vol)
        etabli = [r for r in vol if r['commande'] == cmax]
        if len(etabli) < 5:
            etabli = vol[max(0, int(len(vol) * 0.75)):]
        ecarts = [max(r[f'm{i}'] for i in (1, 2, 3, 4))
                  - min(r[f'm{i}'] for i in (1, 2, 3, 4)) for r in etabli]
        s['ecart_median'] = int(_mediane(ecarts))
        s['ecart_mini'] = int(min(ecarts))
        s['ecart_maxi'] = int(max(ecarts))
        s['ecart_transitoire'] = int(max(
            max(r[f'm{i}'] for i in (1, 2, 3, 4))
            - min(r[f'm{i}'] for i in (1, 2, 3, 4)) for r in vol))
        dernier = etabli[-1]
        s['moteurs_au_pic'] = {f'm{i}': int(dernier[f'm{i}']) for i in (1, 2, 3, 4)}

    # Optical flow. Zeros everywhere means the module is absent or blind.
    if flux:
        s['squal_median'] = round(_mediane([r['squal'] for r in flux]), 1)
        s['flux_abs_max'] = round(max(max(abs(r['deltaX']), abs(r['deltaY']))
                                      for r in flux), 1)
        if 'outlierCount' in flux[0]:
            s['outliers_max'] = int(max(r['outlierCount'] for r in flux))
        if 'std' in flux[0]:
            s['flux_std_median'] = round(_mediane([r['std'] for r in flux]), 1)
        if s['squal_median'] == 0 and s['flux_abs_max'] == 0:
            s['drapeaux'].append('flux_aveugle')
    return s


def resumer(dossier):
    """Write summary.json and print it in a readable form."""
    s = resume(dossier)
    with open(os.path.join(dossier, 'summary.json'), 'w', encoding='utf-8') as f:
        json.dump(s, f, indent=2, ensure_ascii=False)

    p = s['points']
    print(f"\n{s['dossier']}  |  {p['moteurs']} pts moteurs,"
          f" {p['etat']} etat, {p['flux']} flux,"
          f" {p.get('analyses', 0)} analyses (montee + palier)")

    if 'commande_max' in s:
        print(f"  commande max {s['commande_max']}, thrust applique"
              f" {s['thrust_max']:.0f}, ecart {s['ecart_collectif']:+.0f}")
        print(f"  PWM moteur max {s['pwm_max']} / {PWM_MAX}")

    if 'vbat_depart' in s:
        print(f"  batterie {s['vbat_depart']:.2f} -> {s['vbat_min']:.2f} V"
              f"  (chute {s['vbat_chute']:.2f} V, moyenne {s['vbat_moyenne']:.2f} V)")
    if 'hauteur_mm_max' in s:
        print(f"  hauteur {s['hauteur_mm_min']:.0f} -> {s['hauteur_mm_max']:.0f} mm")

    if 'ecart_median' in s:
        m = s['moteurs_au_pic']
        print(f"  au pic  m1={m['m1']}  m2={m['m2']}  m3={m['m3']}  m4={m['m4']}")
        print(f"  ecart median {s['ecart_median']}"
              f" (mini {s['ecart_mini']}, maxi {s['ecart_maxi']},"
              f" transitoire {s['ecart_transitoire']})")
        print('  ' + verdict_equilibrage(s))

    if 'squal_median' in s:
        print(f"  flux squal median {s['squal_median']:.0f},"
              f" deplacement max {s['flux_abs_max']:.0f}")
    if s['drapeaux']:
        print(f"  drapeaux : {', '.join(s['drapeaux'])}")
    return s


def verdict_equilibrage(s):
    """No verdict below VBAT_VERDICT: the spread grows as voltage drops at
    constant balance, because more PWM is needed for the same torque. And none
    at all on the ground: measured 1522 planted and 16955 half unloaded on the
    same ramp, neither comparable to flight."""
    if s.get('mode') != 'vol':
        return ('equilibrage : PAS DE VERDICT, run au sol'
                ' (ne se mesure qu en vol)')
    v = s.get('vbat_moyenne')
    if v is None:
        return 'equilibrage : pas de tension enregistree, aucun verdict'
    if v < safety.VBAT_VERDICT:
        return (f'equilibrage : PAS DE VERDICT a {v:.2f} V'
                f' (sous {safety.VBAT_VERDICT:.1f} V, non comparable)')
    e = s['ecart_median']
    if e > 6000:
        return f'equilibrage : ecart important a {v:.2f} V, centre de gravite decale'
    if e > 3000:
        return f'equilibrage : ecart modere a {v:.2f} V'
    return f'equilibrage : correct a {v:.2f} V'


def planche(dossier):
    """One PNG per run: voltage, motors, attitude, height, flow, trajectory."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    moteurs = lire_csv(os.path.join(dossier, 'moteurs.csv'))
    etat = lire_csv(os.path.join(dossier, 'etat.csv'))
    flux = lire_csv(os.path.join(dossier, 'flux.csv'))
    if not moteurs:
        print('[AVERTISSEMENT] pas de moteurs.csv, planche non produite')
        return None

    fig, ax = plt.subplots(2, 3, figsize=(16, 8), dpi=100)
    fig.suptitle(os.path.basename(dossier))

    t = [r['t_s'] for r in moteurs]
    a = ax[0][0]
    for i in (1, 2, 3, 4):
        a.plot(t, [r[f'm{i}'] for r in moteurs], lw=0.9, label=f'm{i}')
    a.plot(t, [r['commande'] for r in moteurs], 'k--', lw=1.2, label='commande')
    a.set_xlabel('temps (s)'); a.set_ylabel('PWM (0-65535)'); a.legend(fontsize=7)
    a.set_title('moteurs et commande')

    a = ax[0][1]
    a.plot(t, [max(r[f'm{i}'] for i in (1, 2, 3, 4))
               - min(r[f'm{i}'] for i in (1, 2, 3, 4)) for r in moteurs], lw=0.9)
    a.set_xlabel('temps (s)'); a.set_ylabel('ecart max-min (PWM)')
    a.set_title('ecart entre moteurs')

    a = ax[0][2]
    if etat:
        a.plot([r['t_s'] for r in etat], [r['vbat'] for r in etat], lw=1.1)
        a.axhline(safety.VBAT_PLANCHER, color='r', ls=':', lw=1,
                  label=f'plancher {safety.VBAT_PLANCHER}')
        a.axhline(safety.VBAT_COMPARABLE, color='orange', ls=':', lw=1,
                  label=f'comparable {safety.VBAT_COMPARABLE}')
        a.legend(fontsize=7)
    a.set_xlabel('temps (s)'); a.set_ylabel('tension (V)'); a.set_title('batterie')

    a = ax[1][0]
    if etat:
        a.plot([r['t_s'] for r in etat], [r['roll'] for r in etat], lw=0.9, label='roll')
        a.plot([r['t_s'] for r in etat], [r['pitch'] for r in etat], lw=0.9, label='pitch')
        a.legend(fontsize=7)
    a.set_xlabel('temps (s)'); a.set_ylabel('angle (deg)'); a.set_title('assiette')

    a = ax[1][1]
    if etat:
        a.plot([r['t_s'] for r in etat], [r.get('zrange', 0) for r in etat], lw=0.9)
    a.set_xlabel('temps (s)'); a.set_ylabel('hauteur (mm)')
    a.set_title('hauteur (VL53L1X)')

    a = ax[1][2]
    if flux and any(r.get('x') or r.get('y') for r in flux):
        a.plot([r['x'] for r in flux], [r['y'] for r in flux], lw=0.9)
        a.set_xlabel('x (m)'); a.set_ylabel('y (m)')
    else:
        a.text(0.5, 0.5, 'pas de flux\n(module absent ?)', ha='center',
               va='center', fontsize=9)
        a.set_xticks([]); a.set_yticks([])
    a.set_title('trajectoire estimee')

    fig.tight_layout()
    chemin = os.path.join(dossier, 'planche.png')
    fig.savefig(chemin)
    plt.close(fig)
    print(f'Planche -> {chemin}')
    return chemin
