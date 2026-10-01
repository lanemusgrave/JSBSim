"""Unit conversions.  JSBSim works internally in English units (ft, slug, lbf, s).

Property names carry their units as a suffix (``-ft``, ``-fps``, ``-kts``,
``-rad``, ``-deg``, ``-psf``...).  When in doubt, read the suffix.
"""

import math

FT2M = 0.3048
M2FT = 1.0 / FT2M
KT2FPS = 1.6878098571
FPS2KT = 1.0 / KT2FPS
MPS2KT = 1.9438444924
KT2MPS = 1.0 / MPS2KT
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
SLUG2KG = 14.5939029
KG2SLUG = 1.0 / SLUG2KG
LBF2N = 4.4482216153
N2LBF = 1.0 / LBF2N
LB2KG = 0.45359237
SLUGFT2_2_KGM2 = 1.3558179619  # slug*ft^2 -> kg*m^2
PSF2PA = 47.880258889
G0_FPS2 = 32.174049
G0_MPS2 = 9.80665
RHO0_SLUGFT3 = 0.0023769  # sea-level standard density
RHO0_KGM3 = 1.225
