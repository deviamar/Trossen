"""A fake headset. No hardware, no app, no network.

The whole teleop chain -- driver, mapping, governor, arm agent -- can be brought
up and watched with this, which matters because every other way of testing it
involves standing in a room wearing a headset next to a robot that might move.
Bring the chain up on `sim` first, watch the topics, then swap the backend.

    ./launch-quest.sh --backend sim              # slow circle, nothing pressed
    ./launch-quest.sh --backend sim --press right_primary

Motion is a 10 cm circle at 0.2 Hz in front of the operator: small enough to be
safe if it does reach an arm, large enough to see on a plot.
"""
import math
import time

from . import Frame, Hand


class SimBackend:
    def __init__(self, press=(), radius=0.10, period=5.0, **_):
        self.press = set(press or ())
        self.radius = float(radius)
        self.period = float(period)
        self.t0 = time.monotonic()

    def start(self):
        held = ", ".join(sorted(self.press)) or "nothing"
        print(f"  sim backend: {self.radius * 100:.0f} cm circle, "
              f"{self.period:.1f} s period, holding {held}")
        print("  NOTHING IS READING A REAL HEADSET.")

    def read(self):
        t = time.monotonic() - self.t0
        a = 2.0 * math.pi * t / self.period
        # In front (x), swinging left-right (y), bobbing (z). Roughly where a
        # pair of hands would be.
        dy = self.radius * math.cos(a)
        dz = self.radius * math.sin(a)

        def hand(side):
            trig = 1.0 if f"{side}_trigger" in self.press else 0.0
            return Hand(
                position=(0.35, (0.2 if side == "left" else -0.2) + dy, 1.0 + dz),
                orientation=(0.0, 0.0, 0.0, 1.0),
                axes=(0.0, 0.0, trig, 0.0),
                buttons=(1 if f"{side}_primary" in self.press else 0, 0, 0, 0),
                tracked=True,
            )

        return Frame(
            head_position=(0.0, 0.0, 1.6),
            head_orientation=(0.0, 0.0, 0.0, 1.0),
            left=hand("left"),
            right=hand("right"),
        )

    def stop(self):
        pass
