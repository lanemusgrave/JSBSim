"""The airplane that was actually built.  DON'T READ THIS until the debrief.

It differs from the design model (gnc_trainer, all uncertainty multipliers = 1),
the way real airplanes do: control surfaces less effective than the handbook
estimate, more drag, a bigger fin.  Your job in the capstone is to find out
from flight-test data, not from this file.
"""

AS_BUILT = {
    "scale_Cmde": 0.80,     # elevator power: gap seals left off
    "scale_Cmq": 0.80,      # pitch damping
    "scale_Clda": 0.75,     # aileron power: aileron flex + gap
    "scale_Clp": 0.90,      # roll damping
    "scale_Cnb": 1.20,      # weathercock stability: the fin grew in detail design
    "scale_CD": 1.30,       # drag: antennas, landing skids, rough paint
}
