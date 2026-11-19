#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Liste les parametres du firmware lies aux moteurs (aucun moteur demarre)."""
import sys, time, threading, logging
import cflib.crtp
from cflib.crazyflie import Crazyflie

URI = 'udp://192.168.43.42'
logging.basicConfig(level=logging.ERROR)

connecte = threading.Event()
cf = Crazyflie(rw_cache='./cache')
cf.connected.add_callback(lambda uri: connecte.set())

try:
    cflib.crtp.init_drivers()
    cf.open_link(URI)
    if not connecte.wait(10):
        print('[ERREUR] pas de connexion'); sys.exit(1)
    time.sleep(3)  # laisser le TOC se telecharger

    toc = cf.param.toc.toc
    print(f'--- {len(toc)} groupes de parametres ---')
    for grp in sorted(toc):
        if any(k in grp.lower() for k in ('motor', 'power', 'health', 'system')):
            print(f'[{grp}]')
            for name in sorted(toc[grp]):
                print(f'    {grp}.{name}')
    print('--- tous les groupes ---')
    print(', '.join(sorted(toc)))
except Exception as e:
    print(f'[ERREUR] {e}')
finally:
    try: cf.close_link()
    except Exception: pass
