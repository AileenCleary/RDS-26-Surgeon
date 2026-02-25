from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from rds_finger.model import FingerModel
from rds_finger.analysis.routing.length import fixed_contact_sign


@dataclass(frozen=True)
class SpoolDirectionCheck:
    tendon: str
    drum: str | None
    fixed_sign: float | None
    drum_direction: float | None
    ok: bool
    note: str


def check_spool_directions(model: FingerModel) -> list[SpoolDirectionCheck]:
    tendon_to_drum: dict[str, str] = {}
    tendon_to_dir: dict[str, float] = {}

    for dname, d in model.drums.items():
        tendon_to_drum.setdefault(d.tendon, dname)
        tendon_to_dir.setdefault(d.tendon, float(d.direction))

    out: list[SpoolDirectionCheck] = []
    for tname in model.tendon_order:
        spec = model.tendons[tname]
        fsign = fixed_contact_sign(spec)

        dname = tendon_to_drum.get(tname)
        ddir = tendon_to_dir.get(tname)

        if fsign is None or ddir is None:
            out.append(SpoolDirectionCheck(
                tendon=tname, drum=dname, fixed_sign=fsign, drum_direction=ddir,
                ok=True,
                note="no fixed contacts or no drum; nothing to check",
            ))
            continue

        ok = (np.sign(fsign) == np.sign(ddir))
        out.append(SpoolDirectionCheck(
            tendon=tname, drum=dname, fixed_sign=float(np.sign(fsign)), drum_direction=float(np.sign(ddir)),
            ok=bool(ok),
            note="match" if ok else "MISMATCH: fixed contact sign differs from drum.direction",
        ))

    return out