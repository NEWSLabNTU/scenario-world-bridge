"""Issue 091: the state a command leaves is the last one the relay sees before its reply.

`scensim-agent check` failed once under load ("stop: STOPPED before the reply -- phase IDLE"):
the push loop had built a state before `stop` ran and sent it after the command's own STOPPED
state, ahead of the reply. The relay answers with what it shows, so it would have shown the old
phase too. Here that interleave is forced, not waited for under load.
"""

import threading

import scensim_agent
from scensim_agent import protocol as P
from scensim_agent.client import Client


class Stopping(scensim_agent.Agent):
    def on_goal(self, goal, waypoints):
        pass

    def on_stop(self):
        pass

    def on_speed_limit(self, mps):
        pass


class Wire:
    """A connected socket's sendall: the messages, decoded, in the order sent."""

    def __init__(self):
        self.sent = []
        self.reader = P.LineReader()

    def sendall(self, data):
        for line in self.reader.feed(data):
            self.sent.append(P.decode(line))

    def shutdown(self, how):
        pass

    def close(self):
        pass


def test_a_state_built_before_a_command_never_lands_between_its_state_and_reply():
    agent = Stopping()
    c = Client(agent, "127.0.0.1", 1)
    wire = Wire()
    c._sock = wire
    worker_state_sent, stale_sent, push_building = (threading.Event() for _ in range(3))
    send = c._send

    def interleaving_send(message):
        me = threading.current_thread()
        if me.name == "push" and message["type"] == P.STATE:
            push_building.set()
            worker_state_sent.wait(1.0)   # sent after the command's own state, if it can be
            send(message)
            stale_sent.set()
            return
        if me is c._worker and message["type"] == P.STATE:
            send(message)
            worker_state_sent.set()
            stale_sent.wait(0.5)          # gives the stale state its chance before the reply
            return
        send(message)
    c._send = interleaving_send

    push = threading.Thread(target=c.push_state, name="push")
    push.start()
    assert push_building.wait(2.0)        # the push loop has built its (IDLE) state
    c._commands.put((wire, P.command(7, P.STOP)))
    push.join(5.0)
    for _ in range(100):
        if any(m["type"] == P.REPLY for m in wire.sent):
            break
        threading.Event().wait(0.02)
    c.stop()
    c._commands.put(None)
    kinds = [(m["type"], m.get("phase")) for m in wire.sent]
    reply = next(i for i, m in enumerate(wire.sent) if m["type"] == P.REPLY)
    last_state = [m for m in wire.sent[:reply] if m["type"] == P.STATE][-1]
    assert last_state["phase"] == P.STOPPED, kinds
    assert kinds[0] == (P.STATE, P.IDLE)  # the stale state went first
