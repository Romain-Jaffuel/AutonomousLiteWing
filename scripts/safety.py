"""Single source for this drone's safety limits and shared keyboard abort.

Every script imports its limits from here. They used to be redefined per file,
with THRUST_MINI at 20000 in one script and 10001 in another, and three
different ceilings. These values are the operator's, do not change them.
"""
import os
import sys

IP = '192.168.43.42'
URI = f'udp://{IP}'

# Thrust, on the firmware's 0-65535 scale.
THRUST_MINI = 20000         # floor for ramps; the firmware itself accepts 10001
PLAFOND = 60000             # hard ceiling, any request above is refused
CIBLE_DECELERATION = 20000  # keyboard deceleration stops here, then cut

# Key x sheds this fraction over this many seconds, then cuts. Not a dead cut:
# that drops the drone from a hover.
BAISSE_X = 0.2
DUREE_BAISSE_X = 1.0

# Constant tilt in degrees used to cancel a systematic drift. 1 degree is
# 0.17 m/s2 of lateral acceleration, so a typo here would flip the drone.
TRIM_MAX = 10.0

# Battery, in volts, measured under load. The official library only reacts at
# 3.0 V and auto-lands at 2.8 V, far below this pack's loaded floor.
VBAT_PLANCHER = 3.40     # cut flight below this
VBAT_VERDICT = 3.70      # below this, no balance verdict: the motor spread
                         # grows as voltage drops at constant balance
VBAT_COMPARABLE = 3.90   # below this, thrust figures are not comparable
                         # between runs

VOL_MAX_S = 20.0         # the library does not bound flight duration
MARGE_VOL_S = 3.0        # grace before the supervisor calls it overdue
HAUTEUR_MAX_M = 1.20     # commanded height ceiling; the library has none
PAS_SETPOINT = 0.05      # 20 Hz, above the firmware's ~500 ms watchdog
TIMEOUT_CONNEXION = 10.0

TOUCHES_COUPURE = (b'x', b'X', b'\x1b')


class Refus(Exception):
    """A requested value falls outside the limits above."""


def verifier_poussee(*valeurs):
    for v in valeurs:
        if v > PLAFOND:
            raise Refus(f'poussee {v} au-dela du plafond {PLAFOND}')
        if v < THRUST_MINI:
            raise Refus(f'poussee {v} sous le plancher {THRUST_MINI}')


def verifier_trim(roll, pitch):
    for nom, v in (('roll', roll), ('pitch', pitch)):
        if abs(v) > TRIM_MAX:
            raise Refus(f'trim {nom} {v:+.2f} au-dela de +/-{TRIM_MAX:.0f} deg')


def verifier_duree(duree, limite=VOL_MAX_S, nom='duree'):
    if duree <= 0:
        raise Refus(f'{nom} doit etre strictement positive')
    if duree > limite:
        raise Refus(f'{nom} {duree:.1f} s au-dela de {limite:.0f} s')


def borner(valeur):
    """Clamp to the allowed thrust window, for use inside a control loop."""
    return int(max(0, min(PLAFOND, valeur)))


# --- non-blocking keyboard --------------------------------------------------
if os.name == 'nt':
    import msvcrt

    def lire_touche():
        """Pressed key as bytes, or None. Never blocks. Returns None rather
        than raising when there is no console, e.g. output redirected."""
        try:
            if msvcrt.kbhit():
                t = msvcrt.getch()
                while msvcrt.kbhit():
                    msvcrt.getch()
                return t
        except Exception:
            return None
        return None

    def mode_direct_tty():
        pass

    def restaurer_tty():
        pass
else:
    import select
    import termios
    import tty

    _etat_tty = None

    def mode_direct_tty():
        global _etat_tty
        if sys.stdin.isatty():
            _etat_tty = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

    def restaurer_tty():
        if _etat_tty is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, _etat_tty)

    def lire_touche():
        if not sys.stdin.isatty():
            return None
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1).encode()
        return None
