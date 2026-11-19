"""Read the camera's MJPEG stream and measure what it costs the drone link.

The camera joins the drone's own Wi-Fi, so its traffic competes with flight
control on one radio. The point of this module is not the picture: it is to
measure the drone's round-trip time with the stream running and compare it to
the same measurement without, which is what decides whether one Wi-Fi adapter
is enough.

Never connects to the drone over CRTP: only ICMP, so it can run beside a flight.
"""
import os
import re
import socket
import subprocess
import sys
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import safety

SOI, EOI = b'\xff\xd8', b'\xff\xd9'      # JPEG start and end markers
# Parse frames by markers rather than by multipart boundary: servers differ in
# how they spell the boundary, the markers are in the JPEG standard.

PING_RE = re.compile(rb'(?:temps|time)[=<]\s*(\d+)\s*ms', re.I)


def ping(cible=safety.IP, n=10):
    """Returns (sent, received, min, avg, max) in ms. Parses reply lines
    rather than the summary, which is localised."""
    try:
        out = subprocess.run(['ping', '-n', str(n), cible],
                             capture_output=True, timeout=n * 2 + 15).stdout
    except Exception:
        return n, 0, None, None, None
    t = [int(m) for m in PING_RE.findall(out)]
    if not t:
        return n, 0, None, None, None
    return n, len(t), min(t), sum(t) / len(t), max(t)


def lire_flux(url, duree, dossier=None, max_images=3):
    """Pull an MJPEG stream for `duree` seconds.

    Returns a dict: frames, fps, ecart_max (the longest gap between frames,
    which is what a stall looks like), octets, reconnexions, erreur.
    """
    fin = time.time() + duree
    frames, octets, reconnexions = 0, 0, 0
    horodatages, erreur, ecrits = [], None, 0
    tampon = b''

    while time.time() < fin:
        try:
            flux = urllib.request.urlopen(url, timeout=5)
            while time.time() < fin:
                bloc = flux.read(4096)
                if not bloc:
                    break
                octets += len(bloc)
                tampon += bloc
                while True:
                    d = tampon.find(SOI)
                    f = tampon.find(EOI, d + 2) if d >= 0 else -1
                    if d < 0 or f < 0:
                        break
                    image = tampon[d:f + 2]
                    tampon = tampon[f + 2:]
                    frames += 1
                    horodatages.append(time.time())
                    if dossier and ecrits < max_images:
                        os.makedirs(dossier, exist_ok=True)
                        with open(os.path.join(dossier,
                                               f'image{ecrits}.jpg'), 'wb') as g:
                            g.write(image)
                        ecrits += 1
                # a stream that never yields a marker would grow forever
                if len(tampon) > 4_000_000:
                    tampon = b''
        except Exception as e:
            erreur = f'{type(e).__name__}: {e}'
            reconnexions += 1
            if time.time() >= fin:
                break
            time.sleep(0.5)

    ecoule = duree
    ecarts = [b - a for a, b in zip(horodatages, horodatages[1:])]
    return {'frames': frames,
            'fps': round(frames / ecoule, 2) if ecoule else 0,
            'ecart_max': round(max(ecarts), 3) if ecarts else None,
            'octets': octets,
            'debit_kbs': round(octets / ecoule / 1024, 1) if ecoule else 0,
            'reconnexions': reconnexions,
            'images_ecrites': ecrits,
            'erreur': erreur}


def joignable(hote, port, timeout=2.0):
    try:
        with socket.create_connection((hote, port), timeout):
            return True
    except Exception:
        return False
