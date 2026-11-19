#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LiteWing - Test de connexion seul (AUCUN moteur n'est demarre).
Verifie que le lien UDP vers le drone s'ouvre et se ferme proprement.

Usage : python check_link.py
"""
import sys
import time
import logging
import threading

import cflib.crtp
from cflib.crazyflie import Crazyflie

URI = 'udp://192.168.43.42'
TIMEOUT_CONNEXION = 10.0  # secondes

logging.basicConfig(level=logging.ERROR)


class TestConnexion:
    def __init__(self, uri):
        self.uri = uri
        self.connecte = threading.Event()
        self.termine = threading.Event()
        self.erreur = None

        self.cf = Crazyflie(rw_cache='./cache')
        self.cf.connected.add_callback(self._on_connected)
        self.cf.connection_failed.add_callback(self._on_failed)
        self.cf.connection_lost.add_callback(self._on_lost)
        self.cf.disconnected.add_callback(self._on_disconnected)

    def _on_connected(self, uri):
        print(f'[OK] Connecte a {uri}')
        print('     -> la LED bleue LINK doit clignoter sur le drone.')
        self.connecte.set()

    def _on_failed(self, uri, msg):
        self.erreur = f'Connexion refusee : {msg}'
        self.termine.set()

    def _on_lost(self, uri, msg):
        self.erreur = f'Lien perdu : {msg}'
        self.termine.set()

    def _on_disconnected(self, uri):
        print(f'[--] Deconnecte de {uri}')
        self.termine.set()

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

        print('Lien etabli. Maintien 3 s (aucune commande moteur envoyee)...')
        time.sleep(3.0)

        print('Fermeture du lien...')
        self.cf.close_link()
        self.termine.wait(5.0)

        if self.erreur:
            print(f'[AVERTISSEMENT] {self.erreur}')
        print('Termine.')
        return 0


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    sys.exit(TestConnexion(URI).run())
