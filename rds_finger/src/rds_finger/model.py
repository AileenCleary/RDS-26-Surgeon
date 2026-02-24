from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np
from rds_finger import config
from numpy.typing import NDArray
from rds_finger.core.frames import Pose, Line3
from rds_finger.core.math3d import unit
from rds_finger.kinematics.chain import compute_frames
from rds_finger.routing.types import (
    ShaftSpec,
    PulleySpec,
    AnchoredPoint,
    MotorDrumSpec,
    TendonPathSpec,
    TendonContact,
)
from rds_finger.routing.types import BearingSpec, WorldBearing  

@dataclass
class WorldShaft:
    name: str
    axis: Line3
    diameter: float
    length: float

@dataclass
class WorldPulley:
    name: str
    center: NDArray[np.float64]
    axis: NDArray[np.float64]
    radius: float
    width: float

@dataclass
class WorldDrum:
    name: str
    center: NDArray[np.float64]
    axis: NDArray[np.float64]
    radius: float
    tendon: str
    direction: float

@dataclass
class WorldEndpoint:
    name: str
    p: NDArray[np.float64]

@dataclass
class FingerModel:
    link_lengths: dict
    coupling_ratio: float
    shafts: dict[str, ShaftSpec]
    pulleys: dict[str, PulleySpec]
    drums: dict[str, MotorDrumSpec]
    endpoints: dict[str, AnchoredPoint]
    tendons: dict[str, TendonPathSpec]
    tendon_order: list[str]
    bearings: dict[str, BearingSpec]

    # NEW: fingertip definition (defaults chosen to be non-breaking)
    fingertip_parent_frame: str = "O3"
    fingertip_offset_local: NDArray[np.float64] = field(default_factory=lambda: np.array([0.0, 0.0, 0.0], dtype=float))

    def validate(self) -> None:
        # validate tendon refs
        for tname, t in self.tendons.items():
            for item in t.items:
                if isinstance(item, str):
                    kind, name = item.split(":")
                    if kind == "DRUM":
                        if name not in self.drums: raise KeyError(f"{tname} references missing drum {name}")
                    elif kind == "EP":
                        if name not in self.endpoints: raise KeyError(f"{tname} references missing endpoint {name}")
                    else:
                        raise ValueError(f"Unknown tendon item tag: {item}")
                elif isinstance(item, TendonContact):
                    if item.pulley not in self.pulleys:
                        raise KeyError(f"{tname} references missing pulley {item.pulley}")
                else:
                    raise TypeError(f"Unknown tendon item type: {type(item)}")
                # validate bearing refs
        for bname, b in self.bearings.items():
            if b.shaft not in self.shafts:
                raise KeyError(f"Bearing '{bname}' references missing shaft '{b.shaft}'.")
            if float(b.C) <= 0.0:
                raise ValueError(f"Bearing '{bname}' has nonpositive C={b.C}.")
            if float(b.p) <= 0.0:
                raise ValueError(f"Bearing '{bname}' has nonpositive p={b.p}.")

    def world_state(self, q: NDArray[np.float64]):
        """Compute world frames and resolved world objects."""
        self.validate()
        frames = compute_frames(q, link_lengths=self.link_lengths, coupling_ratio=self.coupling_ratio)

        # shafts: anchor frame gives center, axis_local in that frame
        wshafts: dict[str, WorldShaft] = {}
        for s in self.shafts.values():
            F = frames[s.anchor_frame]
            center = F.apply(s.center_local)
            axis = F.R @ unit(s.axis_local)
            wshafts[s.name] = WorldShaft(
                name=s.name,
                axis=Line3(p=center, a=axis),
                diameter=s.diameter,
                length=s.length,
            )
                # bearings: center = shaft center + lane*(shaft axis)
        wbearing: dict[str, WorldBearing] = {}
        for b in self.bearings.values():
            ws = wshafts[b.shaft]
            c = ws.axis.p + float(b.lane) * ws.axis.a
            wbearing[b.name] = WorldBearing(
                name=b.name,
                shaft=b.shaft,
                center=c,
                axis=ws.axis.a.copy(),
            )
        # pulleys: center = shaft center + lane*(shaft axis)
        wpulleys: dict[str, WorldPulley] = {}
        for p in self.pulleys.values():
            ws = wshafts[p.shaft]
            c = ws.axis.p + float(p.lane) * ws.axis.a
            wpulleys[p.name] = WorldPulley(
                name=p.name,
                center=c,
                axis=ws.axis.a.copy(),
                radius=float(p.radius),
                width=float(p.width),
            )

        # endpoints
        wendpoints: dict[str, WorldEndpoint] = {}
        for ep in self.endpoints.values():
            F = frames[ep.frame]
            wendpoints[ep.name] = WorldEndpoint(ep.name, F.apply(ep.p_local))

        # drums
        wdrums: dict[str, WorldDrum] = {}
        for d in self.drums.values():
            F = frames[d.anchor_frame]
            c = F.apply(d.center_local)
            a = F.R @ unit(d.axis_local)
            wdrums[d.name] = WorldDrum(
                name=d.name, center=c, axis=a,
                radius=float(d.radius),
                tendon=d.tendon,
                direction=float(d.direction),
            )
                # fingertip pose
        if self.fingertip_parent_frame not in frames:
            raise KeyError(
                f"FingerModel.fingertip_parent_frame='{self.fingertip_parent_frame}' "
                f"not found in frames. Available: {sorted(frames.keys())}"
            )
        Fp = frames[self.fingertip_parent_frame]
        off = np.asarray(self.fingertip_offset_local, float).reshape(3)
        tip_pose = Fp @ Pose.from_translation(off)
        
        return frames, tip_pose, wpulleys, wendpoints, wdrums, wshafts, wbearing
    
def build_model() -> FingerModel:
    return FingerModel(
        link_lengths=config.LINK_LENGTHS,
        coupling_ratio=config.COUPLING_RATIO,
        shafts=config.SHAFTS,
        pulleys=config.PULLEYS,
        drums=config.DRUMS,
        endpoints=config.ENDPOINTS,
        tendons=config.TENDONS,
        tendon_order=config.TENDON_ORDER,
        fingertip_parent_frame="O3",
        fingertip_offset_local=np.array([getattr(config, "FINGERTIP_OFFSET", 0.0), 0.0, 0.0], dtype=float),
        bearings=config.BEARINGS
    )