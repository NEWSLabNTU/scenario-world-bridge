"""SIGINT for swb's own long-lived entry points (roadmap 079, the F1 class; 077 before the RC).

A command started as a script's background job (``cmd &`` in a non-interactive shell)
inherits SIGINT as ignored, and Python keeps an inherited SIG_IGN: it installs its
KeyboardInterrupt handler only when SIGINT was at its default. Such a ``swb run`` ignored its
process group's SIGINT (118's ladder sat 5 minutes after ``kill -INT -<pgid>``). Each
entry point that is stopped by SIGINT takes it back at its start, whatever it inherited:
``signal.default_int_handler``, KeyboardInterrupt, the stop path it already has.
"""
import signal


def take_sigint():
    """SIGINT raises KeyboardInterrupt from here on, even if it was inherited ignored;
    returns the handler it replaces (for a caller that restores it)."""
    return signal.signal(signal.SIGINT, signal.default_int_handler)
