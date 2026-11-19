#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Liste les variables de LOG du firmware (ce que le drone peut renvoyer).
Aucun moteur n'est demarre."""
import sys
import time
import threading
import logging

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
        print('[ERREUR] pas de connexion')
        sys.exit(1)
    time.sleep(4)   # laisser le TOC de log se telecharger

    toc = cf.log.toc.toc
    print(f'--- {len(toc)} groupes de LOG ---')
    interet = ('motor', 'pm', 'stabilizer', 'range', 'motion',
               'stateestimate', 'acc', 'gyro', 'posctl', 'health')
    for grp in sorted(toc):
        if any(k in grp.lower() for k in interet):
            print(f'[{grp}]')
            for nom in sorted(toc[grp]):
                print(f'    {grp}.{nom}')
    print('')
    print('--- tous les groupes ---')
    print(', '.join(sorted(toc)))
except Exception as e:
    print(f'[ERREUR] {e}')
finally:
    try:
        cf.close_link()
    except Exception:
        pass
