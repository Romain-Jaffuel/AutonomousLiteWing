#!/usr/bin/env python3
"""Measure what a detector really costs on this machine, over a flight's length.

Written to be copied to the laptop and run there: that machine is the target,
not the desktop. It builds its own environment and touches nothing else.

The window is one flight, not a soak test. A ten-minute benchmark says little
when a flight lasts two or three minutes; what matters is whether the machine
slows down inside that window, which a 15 W laptop chip may well do. Results
are therefore reported per slice, not as a single average.

    uv venv .venv-vision --python 3.11
    uv pip install --python .venv-vision --index-url https://download.pytorch.org/whl/cpu torch torchvision
    uv pip install --python .venv-vision ultralytics openvino
    .venv-vision/Scripts/python.exe scripts/bench_vision.py

    --duree 180        seconds, one flight (default)
    --taille 320       inference size
    --appareils cpu,intel:cpu,intel:gpu,intel:npu
"""
import argparse
import json
import os
import platform
import statistics
import sys
import time
from datetime import datetime

RACINE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SORTIE = os.path.join(RACINE, 'data', 'bench')


def machine():
    infos = {'os': platform.platform(), 'python': platform.python_version(),
             'processeur': platform.processor(), 'coeurs': os.cpu_count()}
    try:
        import subprocess
        n = subprocess.run(['powershell', '-NoProfile', '-Command',
                            '(Get-CimInstance Win32_Processor).Name'],
                           capture_output=True, text=True, timeout=20).stdout.strip()
        if n:
            infos['processeur'] = n
    except Exception:
        pass
    return infos


def mesurer(modele, image, appareil, duree, taille, tranche=30.0):
    """Run until `duree` elapses, timing each inference.

    Slices keep thermal behaviour visible: a chip that holds 40 fps for the
    first minute and 22 fps afterwards is a different proposition from one
    that holds 30 fps throughout, and an average hides exactly that.
    """
    for _ in range(5):                      # warm-up, excluded
        modele.predict(image, imgsz=taille, device=appareil, verbose=False)

    debut = time.perf_counter()
    temps, tranches, t_tranche = [], [], debut
    courant = []
    while time.perf_counter() - debut < duree:
        t0 = time.perf_counter()
        modele.predict(image, imgsz=taille, device=appareil, verbose=False)
        dt = (time.perf_counter() - t0) * 1000
        temps.append(dt)
        courant.append(dt)
        if time.perf_counter() - t_tranche >= tranche:
            tranches.append(round(1000 / statistics.median(courant), 1))
            courant, t_tranche = [], time.perf_counter()
    if courant:
        tranches.append(round(1000 / statistics.median(courant), 1))

    temps.sort()
    med = statistics.median(temps)
    return {'images': len(temps),
            'ms_median': round(med, 2),
            'ms_p95': round(temps[int(len(temps) * 0.95) - 1], 2),
            'fps_median': round(1000 / med, 1),
            'fps_par_tranche': tranches,
            # how much the end differs from the start: the throttling signal
            'perte_fin_pct': (round(100 * (tranches[0] - tranches[-1])
                                    / tranches[0], 1)
                              if len(tranches) > 1 and tranches[0] else 0.0)}


def main():
    p = argparse.ArgumentParser(description='Banc detection, duree d un vol')
    p.add_argument('--duree', type=float, default=180.0,
                   help='secondes par appareil (defaut 180, soit un vol)')
    p.add_argument('--taille', type=int, default=320)
    p.add_argument('--modele', default='yolo11n.pt')
    p.add_argument('--appareils', default='cpu',
                   help='liste separee par des virgules : cpu, intel:cpu,'
                        ' intel:gpu, intel:npu')
    a = p.parse_args()

    try:
        import numpy as np
        from ultralytics import YOLO
    except ImportError as e:
        print(f'[ERREUR] environnement incomplet : {e}')
        print('Voir les commandes en tete de ce fichier.')
        return 1

    infos = machine()
    print(f"machine : {infos['processeur']}, {infos['coeurs']} coeurs logiques")
    print(f"modele  : {a.modele} a {a.taille} px, {a.duree:.0f} s par appareil\n")

    # A fixed synthetic frame: the point is the compute cost, and a real photo
    # would make the number depend on how many objects happen to be in it.
    image = np.random.randint(0, 255, (a.taille, a.taille, 3), dtype=np.uint8)
    resultats = {}

    for appareil in [d.strip() for d in a.appareils.split(',') if d.strip()]:
        print(f'--- {appareil} ---')
        try:
            m = YOLO(a.modele)
            if appareil.startswith('intel:'):
                # OpenVINO needs an exported model; export once, reuse after.
                dossier = a.modele.replace('.pt', '_openvino_model')
                if not os.path.isdir(dossier):
                    print('  export OpenVINO...')
                    m.export(format='openvino', imgsz=a.taille, int8=False)
                m = YOLO(dossier, task='detect')
            r = mesurer(m, image, appareil, a.duree, a.taille)
            resultats[appareil] = r
            print(f"  {r['fps_median']} fps median, {r['ms_median']} ms"
                  f"  (p95 {r['ms_p95']} ms, {r['images']} images)")
            print(f"  par tranche de 30 s : {r['fps_par_tranche']}")
            if r['perte_fin_pct'] > 10:
                print(f"  /!\\ ralentissement de {r['perte_fin_pct']}% sur la"
                      f" duree : la machine chauffe")
            elif len(r['fps_par_tranche']) > 1:
                print('  tenue stable sur la duree')
        except Exception as e:
            print(f'  indisponible : {type(e).__name__}: {e}')
            resultats[appareil] = {'erreur': f'{type(e).__name__}: {e}'}
        print()

    os.makedirs(SORTIE, exist_ok=True)
    nom = f"{datetime.now():%Y-%m-%d-%H%M}-bench.json"
    with open(os.path.join(SORTIE, nom), 'w', encoding='utf-8') as f:
        json.dump({'machine': infos, 'modele': a.modele, 'taille': a.taille,
                   'duree_s': a.duree, 'resultats': resultats}, f,
                  indent=2, ensure_ascii=False)
    print(f'-> {os.path.join(SORTIE, nom)}')

    # A detection has to fit in the frame budget alongside tracking and the
    # command, so the headline number is the budget left, not the fps.
    ok = [d for d, r in resultats.items() if 'ms_median' in r and r['ms_median'] < 50]
    if ok:
        print(f"\nTiennent sous 50 ms : {', '.join(ok)}")
    else:
        print('\nAucun appareil sous 50 ms : baisser la resolution,'
              ' detecter une image sur deux, ou quantifier en INT8.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
