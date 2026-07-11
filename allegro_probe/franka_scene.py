"""Stage-1 MuJoCo model for a Panda arm carrying an Allegro right hand.

This module intentionally stops at the embodiment boundary.  It compiles the
two unmodified MuJoCo Menagerie source models into one 7+16 DoF model and
offers joint, frame and collision inspection APIs.  It does not add the old
6-DoF carriage, the central probe, task objects, IK, or probe primitives.

The hand mount is synthetic and versioned.  Its transform is useful for
simulation integration tests, but is not a measurement of a physical adapter.
Replacing it later therefore changes a mount profile rather than silently
changing the robot convention.
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Optional, Tuple

import numpy as np

from allegro_probe.models import FRANKA_ALLEGRO_MUJOCO_BACKEND
from allegro_probe.scene import ALLEGRO_ACTUATORS, _ALLEGRO_OPEN


# Backwards-friendly local alias; models.py remains the one backend-id source
# of truth shared by registration, construction and this embodiment module.
FRANKA_ALLEGRO_BACKEND = FRANKA_ALLEGRO_MUJOCO_BACKEND
DEFAULT_MENAGERIE_ROOT = Path("/home/enovo/robots/sim/mujoco_menagerie")
STAGE1_CONTROLLER_PROFILE_ID = "sim.panda_menagerie_pd+allegro_position.v1"

PANDA_JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 8))
PANDA_ACTUATOR_NAMES = tuple(f"actuator{i}" for i in range(1, 8))
ALLEGRO_JOINT_NAMES = (
    "ffj0", "ffj1", "ffj2", "ffj3",
    "mfj0", "mfj1", "mfj2", "mfj3",
    "rfj0", "rfj1", "rfj2", "rfj3",
    "thj0", "thj1", "thj2", "thj3",
)
HAND_JOINT_NAMES = tuple(f"hand/{name}" for name in ALLEGRO_JOINT_NAMES)
HAND_ACTUATOR_NAMES = tuple(f"hand/{name}" for name in ALLEGRO_ACTUATORS)

CANONICAL_PANDA_HOME = np.array(
    [0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, -0.7853], dtype=float
)
CANONICAL_PANDA_HOME.setflags(write=False)
# Freeze the current execution-layer Allegro open pose as this model profile's
# canonical hand pose.  In particular, thj0=0.45 is within its [0.263, 1.396]
# range; the Menagerie all-zero default is not a legal Allegro configuration.
CANONICAL_ALLEGRO_OPEN = np.asarray(_ALLEGRO_OPEN, dtype=float).copy()
CANONICAL_ALLEGRO_OPEN.setflags(write=False)
CANONICAL_Q = np.concatenate((CANONICAL_PANDA_HOME, CANONICAL_ALLEGRO_OPEN))
CANONICAL_Q.setflags(write=False)


@dataclass(frozen=True)
class SyntheticMountProfile:
    """Explicit transform and inertial approximation for the simulation mount."""

    profile_id: str
    attachment_to_palm_position_m: Tuple[float, float, float]
    attachment_to_palm_quaternion_wxyz: Tuple[float, float, float, float]
    adapter_mass_kg: float
    adapter_com_m: Tuple[float, float, float]
    adapter_diagonal_inertia_kg_m2: Tuple[float, float, float]
    adapter_collision_fromto_m: Tuple[float, float, float, float, float, float]
    adapter_collision_radius_m: float

    def __post_init__(self) -> None:
        pos = np.asarray(self.attachment_to_palm_position_m, dtype=float)
        quat = np.asarray(self.attachment_to_palm_quaternion_wxyz, dtype=float)
        inertia = np.asarray(self.adapter_diagonal_inertia_kg_m2, dtype=float)
        fromto = np.asarray(self.adapter_collision_fromto_m, dtype=float)
        if pos.shape != (3,) or not np.all(np.isfinite(pos)):
            raise ValueError("mount position must contain three finite values")
        if quat.shape != (4,) or not np.all(np.isfinite(quat)):
            raise ValueError("mount quaternion must contain four finite values")
        if not np.isclose(np.linalg.norm(quat), 1.0, atol=1e-7):
            raise ValueError("mount quaternion must be unit length")
        if float(self.adapter_mass_kg) <= 0.0:
            raise ValueError("adapter mass must be positive")
        if inertia.shape != (3,) or np.any(inertia <= 0.0):
            raise ValueError("adapter diagonal inertia must be positive")
        if fromto.shape != (6,) or not np.all(np.isfinite(fromto)):
            raise ValueError("adapter collision fromto must contain six finite values")
        if float(self.adapter_collision_radius_m) <= 0.0:
            raise ValueError("adapter collision radius must be positive")


SYNTHETIC_MOUNT_V1 = SyntheticMountProfile(
    profile_id="sim.synthetic_panda_allegro_mount.v1",
    # Panda attachment +Z points down in the canonical home configuration.
    # Allegro fingers extend along palm +Z, so this places the open hand below
    # the flange without inheriting the standalone hand model's root rotation.
    attachment_to_palm_position_m=(0.0, 0.0, 0.120),
    attachment_to_palm_quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
    adapter_mass_kg=0.120,
    adapter_com_m=(0.0, 0.0, 0.0125),
    adapter_diagonal_inertia_kg_m2=(2.6e-5, 2.6e-5, 1.5e-5),
    # The Allegro palm collision box reaches 95 mm behind the palm origin.
    # With the palm at 120 mm, its back face is 25 mm from the attachment.
    # Start 3 mm beyond the attachment origin so the mount has a deliberate
    # positive clearance from link7's collision mesh as well as from the palm.
    adapter_collision_fromto_m=(0.0, 0.0, 0.003, 0.0, 0.0, 0.024),
    adapter_collision_radius_m=0.020,
)


@dataclass(frozen=True)
class FrankaSceneConfig:
    """Compilation profile for :class:`FrankaAllegroScene`."""

    menagerie_root: Path = DEFAULT_MENAGERIE_ROOT
    backend: str = FRANKA_ALLEGRO_BACKEND
    timestep: float = 0.002
    hand_kp: float = 8.0
    mount_profile: SyntheticMountProfile = SYNTHETIC_MOUNT_V1

    def __post_init__(self) -> None:
        object.__setattr__(self, "menagerie_root", Path(self.menagerie_root))
        if self.backend != FRANKA_ALLEGRO_BACKEND:
            raise ValueError(
                f"FrankaSceneConfig.backend must be {FRANKA_ALLEGRO_BACKEND!r}"
            )
        if not np.isfinite(float(self.timestep)) or float(self.timestep) <= 0.0:
            raise ValueError("timestep must be finite and positive")
        if not np.isfinite(float(self.hand_kp)) or float(self.hand_kp) <= 0.0:
            raise ValueError("hand_kp must be finite and positive")


@dataclass(frozen=True)
class FramePose:
    """World pose of a named body/site, using MuJoCo's wxyz convention."""

    position_m: np.ndarray
    quaternion_wxyz: np.ndarray
    rotation_matrix: np.ndarray

    def as_matrix(self) -> np.ndarray:
        transform = np.eye(4, dtype=float)
        transform[:3, :3] = self.rotation_matrix
        transform[:3, 3] = self.position_m
        return transform


@dataclass(frozen=True)
class ModelProvenance:
    """Immutable identity of the two source MJCFs and compilation profiles."""

    menagerie_root: Path
    panda_xml_path: Path
    panda_xml_sha256: str
    allegro_xml_path: Path
    allegro_xml_sha256: str
    mujoco_version: str
    timestep_s: float
    cone: str
    integrator: str
    solver: str
    mount_profile_id: str
    controller_profile_id: str
    hand_kp: float


@dataclass(frozen=True)
class CollisionPair:
    geom1: str
    geom2: str
    body1: str
    body2: str
    signed_distance_m: float
    normal_force_N: float
    policy_filter_reason: str = ""

    @property
    def penetration_m(self) -> float:
        return max(0.0, -float(self.signed_distance_m))

    @property
    def policy_filtered(self) -> bool:
        return bool(self.policy_filter_reason)


@dataclass(frozen=True)
class CollisionSnapshot:
    contacts: Tuple[CollisionPair, ...]

    @property
    def forbidden_contacts(self) -> Tuple[CollisionPair, ...]:
        return tuple(pair for pair in self.contacts if not pair.policy_filtered)

    @property
    def has_self_collision(self) -> bool:
        return bool(self.forbidden_contacts)

    @property
    def max_penetration_m(self) -> float:
        return max((pair.penetration_m for pair in self.contacts), default=0.0)

    @property
    def total_normal_force_N(self) -> float:
        return float(sum(pair.normal_force_N for pair in self.contacts))


@dataclass(frozen=True)
class DistanceAudit:
    """Near robot-robot geom pairs, including pairs filtered by contact policy."""

    search_radius_m: float
    pairs: Tuple[CollisionPair, ...]
    penetration_tolerance_m: float = 1e-7

    @property
    def forbidden_penetrations(self) -> Tuple[CollisionPair, ...]:
        return tuple(
            pair
            for pair in self.pairs
            if not pair.policy_filtered
            and pair.signed_distance_m < -self.penetration_tolerance_m
        )

    @property
    def has_forbidden_penetration(self) -> bool:
        return bool(self.forbidden_penetrations)

    @property
    def minimum_forbidden_distance_m(self) -> float:
        values = [
            pair.signed_distance_m
            for pair in self.pairs
            if not pair.policy_filtered
        ]
        return float(min(values, default=self.search_radius_m))


# The Allegro source model declares these exclusions explicitly.  Direct
# parent-child pairs are classified dynamically in _pair_filter_reason().
_EXPLICIT_FILTERED_BODY_PAIRS = frozenset(
    {
        frozenset(("link0", "link1")),
        frozenset(("hand/palm", "hand/ff_base")),
        frozenset(("hand/palm", "hand/mf_base")),
        frozenset(("hand/palm", "hand/rf_base")),
        frozenset(("hand/palm", "hand/th_base")),
        frozenset(("hand/palm", "hand/th_proximal")),
    }
)
_HAND_INTERNAL_GEOMETRY_PAIRS = frozenset(
    {
        # The Menagerie fingertip capsule intentionally reaches back across
        # the distal body into the medial collision box.  Name only these
        # known two-hop overlaps; never exempt two-hop pairs globally.
        frozenset(("hand/ff_medial", "hand/ff_tip")),
        frozenset(("hand/mf_medial", "hand/mf_tip")),
        frozenset(("hand/rf_medial", "hand/rf_tip")),
    }
)
_SYNTHETIC_MOUNT_INTERFACE = frozenset(("synthetic_mount", "hand/palm"))

_FRAME_SITE_ALIASES = {
    "attachment": "attachment_site",
    "mount_palm": "synthetic_mount_palm_site",
    "allegro_palm": "hand/palm_site",
    "protocol_wrist": "hand/protocol_wrist_site",
    "ff_tip": "hand/ff_tip_site",
    "mf_tip": "hand/mf_tip_site",
    "rf_tip": "hand/rf_tip_site",
    "th_tip": "hand/th_tip_site",
}
_FRAME_BODY_ALIASES = {
    "panda_base": "link0",
    "adapter": "synthetic_mount",
}


class FrankaAllegroScene:
    """Independent 23-DoF Panda+Allegro stage-1 MuJoCo scene."""

    def __init__(
        self,
        task: Any = None,
        config: Optional[FrankaSceneConfig] = None,
    ) -> None:
        try:
            import mujoco  # type: ignore
        except Exception as exc:  # pragma: no cover - environment dependent
            raise RuntimeError("mujoco is required for FrankaAllegroScene") from exc

        if not hasattr(mujoco, "MjSpec"):
            raise RuntimeError("FrankaAllegroScene requires MuJoCo with MjSpec support")

        self.mujoco = mujoco
        self.task = task
        self.config = config or FrankaSceneConfig()
        self.mount_profile = self.config.mount_profile
        self.spec = self._build_spec()
        self.model = self.spec.compile()
        self.data = mujoco.MjData(self.model)
        self.dt = float(self.model.opt.timestep)
        self._model_provenance = self._make_model_provenance()
        self._index()
        self._validate_compiled_model()
        self.reset()

    # ------------------------------------------------------------------ model assembly
    def _source_paths(self) -> Tuple[Path, Path]:
        root = Path(self.config.menagerie_root)
        panda = root / "franka_emika_panda" / "panda_nohand.xml"
        hand = root / "wonik_allegro" / "right_hand.xml"
        for label, path in (("Panda", panda), ("Allegro", hand)):
            if not path.is_file():
                raise FileNotFoundError(f"MuJoCo Menagerie {label} model not found: {path}")
        return panda, hand

    @staticmethod
    def _file_sha256(path: Path) -> str:
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()

    def _make_model_provenance(self) -> ModelProvenance:
        panda_path, hand_path = self._source_paths()
        mj = self.mujoco
        return ModelProvenance(
            menagerie_root=Path(self.config.menagerie_root).resolve(),
            panda_xml_path=panda_path.resolve(),
            panda_xml_sha256=self._file_sha256(panda_path),
            allegro_xml_path=hand_path.resolve(),
            allegro_xml_sha256=self._file_sha256(hand_path),
            mujoco_version=str(mj.__version__),
            timestep_s=float(self.model.opt.timestep),
            cone=mj.mjtCone(int(self.model.opt.cone)).name.removeprefix(
                "mjCONE_"
            ).lower(),
            integrator=mj.mjtIntegrator(int(self.model.opt.integrator)).name.removeprefix(
                "mjINT_"
            ).lower(),
            solver=mj.mjtSolver(int(self.model.opt.solver)).name.removeprefix(
                "mjSOL_"
            ).lower(),
            mount_profile_id=self.mount_profile.profile_id,
            controller_profile_id=STAGE1_CONTROLLER_PROFILE_ID,
            hand_kp=float(self.config.hand_kp),
        )

    @property
    def model_provenance(self) -> ModelProvenance:
        return self._model_provenance

    @staticmethod
    def _name_collision_geoms(spec: Any, prefix: str) -> None:
        used = {geom.name for geom in spec.geoms if geom.name}
        for index, geom in enumerate(spec.geoms):
            if not (int(geom.contype) or int(geom.conaffinity)) or geom.name:
                continue
            body = str(geom.parent.name or "body")
            class_name = str(getattr(geom.classname, "name", "") or "collision")
            detail = str(geom.meshname or class_name or f"geom{index}")
            base = f"{prefix}{body}_{detail}"
            name = base
            suffix = 2
            while name in used:
                name = f"{base}_{suffix}"
                suffix += 1
            geom.name = name
            used.add(name)

    def _build_spec(self) -> Any:
        mj = self.mujoco
        panda_path, hand_path = self._source_paths()
        panda = mj.MjSpec.from_file(str(panda_path))
        hand = mj.MjSpec.from_file(str(hand_path))

        # Child mesh paths would otherwise be resolved relative to the Panda
        # compiler meshdir after attachment.  Absolute paths keep each source
        # model tied to its own Menagerie asset tree.
        hand_asset_root = hand_path.parent / str(hand.meshdir)
        for mesh in hand.meshes:
            mesh.file = str((hand_asset_root / str(mesh.file)).resolve())

        panda.modelname = "panda_allegro_stage1"
        panda.option.timestep = float(self.config.timestep)
        # Allegro requests elliptic friction.  Set it on both specs before
        # attachment so MjSpec has no parent/child option conflict to resolve.
        panda.option.cone = mj.mjtCone.mjCONE_ELLIPTIC
        hand.option.cone = mj.mjtCone.mjCONE_ELLIPTIC
        self._name_collision_geoms(panda, "arm/")
        self._name_collision_geoms(hand, "")

        for actuator_name in ALLEGRO_ACTUATORS:
            actuator = hand.actuator(actuator_name)
            actuator.gainprm[0] = float(self.config.hand_kp)
            actuator.biasprm[1] = -float(self.config.hand_kp)

        tip_sites = {
            "ff_tip": ((0.0, 0.0, 0.030), 0.004),
            "mf_tip": ((0.0, 0.0, 0.030), 0.004),
            "rf_tip": ((0.0, 0.0, 0.030), 0.004),
            "th_tip": ((0.0, 0.0, 0.044), 0.004),
        }
        for body_name, (position, radius) in tip_sites.items():
            hand.body(body_name).add_site(
                name=f"{body_name}_site",
                pos=position,
                size=[radius, 0.0, 0.0],
                rgba=[0.05, 0.65, 1.0, 0.45],
            )
        hand.body("palm").add_site(
            name="palm_site", size=[0.003, 0.0, 0.0], rgba=[0.9, 0.3, 0.1, 0.5]
        )
        hand.body("palm").add_site(
            name="protocol_wrist_site",
            size=[0.003, 0.0, 0.0],
            rgba=[0.2, 0.9, 0.2, 0.5],
        )

        profile = self.mount_profile
        attachment = panda.body("attachment")
        adapter = attachment.add_body(
            name="synthetic_mount",
            mass=float(profile.adapter_mass_kg),
            ipos=profile.adapter_com_m,
            inertia=profile.adapter_diagonal_inertia_kg_m2,
            explicitinertial=True,
        )
        adapter.add_geom(
            name="synthetic_mount_collision",
            type=mj.mjtGeom.mjGEOM_CYLINDER,
            fromto=profile.adapter_collision_fromto_m,
            size=[float(profile.adapter_collision_radius_m), 0.0, 0.0],
            density=0.0,
            rgba=[0.16, 0.17, 0.20, 1.0],
            group=3,
        )
        adapter.add_site(
            name="synthetic_mount_palm_site",
            pos=profile.attachment_to_palm_position_m,
            quat=profile.attachment_to_palm_quaternion_wxyz,
            size=[0.003, 0.0, 0.0],
            rgba=[0.9, 0.75, 0.1, 0.5],
        )
        palm_frame = adapter.add_frame(
            name="synthetic_mount_palm_frame",
            pos=profile.attachment_to_palm_position_m,
            quat=profile.attachment_to_palm_quaternion_wxyz,
        )
        attached_palm = palm_frame.attach_body(hand.body("palm"), "hand/", "")
        # right_hand.xml's root quaternion is a standalone display convention.
        # The complete attachment transform is owned by the mount profile.
        attached_palm.pos = [0.0, 0.0, 0.0]
        attached_palm.quat = [1.0, 0.0, 0.0, 0.0]

        home = panda.key("home")
        home.qpos = CANONICAL_Q
        home.ctrl = CANONICAL_Q
        return panda

    def _index(self) -> None:
        mj = self.mujoco
        self.arm_joint_ids = np.array(
            [mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, name)
             for name in PANDA_JOINT_NAMES],
            dtype=int,
        )
        self.hand_joint_ids = np.array(
            [mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, name)
             for name in HAND_JOINT_NAMES],
            dtype=int,
        )
        self.arm_qpos_indices = self.model.jnt_qposadr[self.arm_joint_ids].astype(int)
        self.hand_qpos_indices = self.model.jnt_qposadr[self.hand_joint_ids].astype(int)
        self.arm_actuator_ids = np.array(
            [mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, name)
             for name in PANDA_ACTUATOR_NAMES],
            dtype=int,
        )
        self.hand_actuator_ids = np.array(
            [mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, name)
             for name in HAND_ACTUATOR_NAMES],
            dtype=int,
        )
        self._collision_geom_ids = np.array(
            [
                geom_id
                for geom_id in range(self.model.ngeom)
                if int(self.model.geom_contype[geom_id])
                or int(self.model.geom_conaffinity[geom_id])
            ],
            dtype=int,
        )
        self._home_key_id = mj.mj_name2id(
            self.model, mj.mjtObj.mjOBJ_KEY, "home"
        )

    def _validate_compiled_model(self) -> None:
        if (self.model.nq, self.model.nv, self.model.nu) != (23, 23, 23):
            raise RuntimeError(
                "Panda+Allegro stage-1 model must compile to nq=nv=nu=23; "
                f"got {(self.model.nq, self.model.nv, self.model.nu)}"
            )
        expected_joints = PANDA_JOINT_NAMES + HAND_JOINT_NAMES
        actual_joints = tuple(
            self.model.joint(index).name for index in range(self.model.njnt)
        )
        if actual_joints != expected_joints:
            raise RuntimeError(f"unexpected 7+16 joint order: {actual_joints!r}")
        expected_actuators = PANDA_ACTUATOR_NAMES + HAND_ACTUATOR_NAMES
        actual_actuators = tuple(
            self.model.actuator(index).name for index in range(self.model.nu)
        )
        if actual_actuators != expected_actuators:
            raise RuntimeError(f"unexpected 7+16 actuator order: {actual_actuators!r}")
        if self._home_key_id < 0:
            raise RuntimeError("compiled model has no canonical 'home' keyframe")
        if not np.all(np.isfinite(CANONICAL_Q)):
            raise RuntimeError("canonical q must contain only finite values")
        if not np.all(self.model.jnt_limited):
            raise RuntimeError("all 23 stage-1 joints must have explicit limits")
        joint_ranges = self.model.jnt_range
        if np.any(CANONICAL_Q < joint_ranges[:, 0]) or np.any(
            CANONICAL_Q > joint_ranges[:, 1]
        ):
            raise RuntimeError("canonical q is outside the compiled joint limits")
        if not np.all(self.model.actuator_ctrllimited):
            raise RuntimeError("all 23 stage-1 actuators must have ctrl limits")
        ctrl_ranges = self.model.actuator_ctrlrange
        if np.any(CANONICAL_Q < ctrl_ranges[:, 0]) or np.any(
            CANONICAL_Q > ctrl_ranges[:, 1]
        ):
            raise RuntimeError("canonical ctrl is outside actuator limits")

    # ------------------------------------------------------------------ state and control
    def reset(self) -> None:
        self.mujoco.mj_resetDataKeyframe(self.model, self.data, self._home_key_id)
        self.data.ctrl[:] = CANONICAL_Q
        self.mujoco.mj_forward(self.model, self.data)

    def step(self, steps: int = 1) -> None:
        if isinstance(steps, bool) or int(steps) != steps or int(steps) <= 0:
            raise ValueError("steps must be a positive integer")
        for _ in range(int(steps)):
            self.mujoco.mj_step(self.model, self.data)

    @property
    def arm_qpos(self) -> np.ndarray:
        return self.data.qpos[self.arm_qpos_indices].copy()

    @property
    def hand_qpos(self) -> np.ndarray:
        return self.data.qpos[self.hand_qpos_indices].copy()

    def _validate_target(
        self,
        values: Iterable[float],
        actuator_ids: np.ndarray,
        expected_size: int,
        label: str,
    ) -> np.ndarray:
        target = np.asarray(values, dtype=float)
        if target.shape != (expected_size,):
            raise ValueError(
                f"{label} target must have shape ({expected_size},), got {target.shape}"
            )
        if not np.all(np.isfinite(target)):
            raise ValueError(f"{label} target must contain only finite values")
        limited = self.model.actuator_ctrllimited[actuator_ids].astype(bool)
        ranges = self.model.actuator_ctrlrange[actuator_ids]
        too_low = limited & (target < ranges[:, 0] - 1e-12)
        too_high = limited & (target > ranges[:, 1] + 1e-12)
        if np.any(too_low | too_high):
            bad = int(np.flatnonzero(too_low | too_high)[0])
            raise ValueError(
                f"{label} target[{bad}]={target[bad]:.6g} is outside actuator "
                f"range [{ranges[bad, 0]:.6g}, {ranges[bad, 1]:.6g}]"
            )
        return target

    def command_arm_joints(self, q7: Iterable[float]) -> np.ndarray:
        target = self._validate_target(q7, self.arm_actuator_ids, 7, "arm")
        self.data.ctrl[self.arm_actuator_ids] = target
        return target.copy()

    def command_hand_joints(self, q16: Iterable[float]) -> np.ndarray:
        target = self._validate_target(q16, self.hand_actuator_ids, 16, "hand")
        self.data.ctrl[self.hand_actuator_ids] = target
        return target.copy()

    # ------------------------------------------------------------------ FK frames
    @property
    def frame_names(self) -> Tuple[str, ...]:
        return tuple(_FRAME_BODY_ALIASES) + tuple(_FRAME_SITE_ALIASES)

    def frame_pose(self, frame_name: str) -> FramePose:
        mj = self.mujoco
        name = str(frame_name)
        body_name = _FRAME_BODY_ALIASES.get(name)
        if body_name is not None:
            body_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, body_name)
            position = self.data.xpos[body_id].copy()
            quaternion = self.data.xquat[body_id].copy()
            rotation = self.data.xmat[body_id].reshape(3, 3).copy()
            return FramePose(position, quaternion, rotation)

        site_name = _FRAME_SITE_ALIASES.get(name, name)
        site_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_SITE, site_name)
        if site_id < 0:
            known = ", ".join(self.frame_names)
            raise KeyError(f"unknown frame {frame_name!r}; known aliases: {known}")
        position = self.data.site_xpos[site_id].copy()
        rotation = self.data.site_xmat[site_id].reshape(3, 3).copy()
        quaternion = np.empty(4, dtype=float)
        mj.mju_mat2Quat(quaternion, rotation.reshape(-1))
        return FramePose(position, quaternion, rotation)

    # ------------------------------------------------------------------ collision audit
    def _geom_name(self, geom_id: int) -> str:
        return self.model.geom(geom_id).name or f"<unnamed_geom_{geom_id}>"

    def _pair_filter_reason(self, body1_id: int, body2_id: int) -> str:
        if body1_id == body2_id:
            return "same_body"
        names = frozenset(
            (self.model.body(body1_id).name, self.model.body(body2_id).name)
        )
        if names == _SYNTHETIC_MOUNT_INTERFACE:
            return "synthetic_mount_interface"
        if names in _HAND_INTERNAL_GEOMETRY_PAIRS:
            return "named_hand_internal_overlap"
        if names in _EXPLICIT_FILTERED_BODY_PAIRS:
            return "source_model_exclude"
        if (
            int(self.model.body_parentid[body1_id]) == body2_id
            or int(self.model.body_parentid[body2_id]) == body1_id
        ):
            return "direct_parent_child"
        return ""

    def collision_snapshot(self) -> CollisionSnapshot:
        records = []
        for contact_index in range(self.data.ncon):
            contact = self.data.contact[contact_index]
            geom1 = int(contact.geom1)
            geom2 = int(contact.geom2)
            body1 = int(self.model.geom_bodyid[geom1])
            body2 = int(self.model.geom_bodyid[geom2])
            wrench = np.zeros(6, dtype=float)
            self.mujoco.mj_contactForce(
                self.model, self.data, contact_index, wrench
            )
            records.append(
                CollisionPair(
                    geom1=self._geom_name(geom1),
                    geom2=self._geom_name(geom2),
                    body1=self.model.body(body1).name,
                    body2=self.model.body(body2).name,
                    signed_distance_m=float(contact.dist),
                    normal_force_N=max(0.0, float(wrench[0])),
                    policy_filter_reason=self._pair_filter_reason(body1, body2),
                )
            )
        records.sort(key=lambda pair: pair.signed_distance_m)
        return CollisionSnapshot(tuple(records))

    def distance_audit(self, max_distance_m: float = 0.050) -> DistanceAudit:
        radius = float(max_distance_m)
        if not np.isfinite(radius) or radius <= 0.0:
            raise ValueError("max_distance_m must be finite and positive")
        records = []
        fromto = np.zeros(6, dtype=float)
        geom_ids = self._collision_geom_ids
        for offset, geom1_value in enumerate(geom_ids[:-1]):
            geom1 = int(geom1_value)
            body1 = int(self.model.geom_bodyid[geom1])
            for geom2_value in geom_ids[offset + 1:]:
                geom2 = int(geom2_value)
                body2 = int(self.model.geom_bodyid[geom2])
                # MuJoCo 3.10 can return raw 0 for some separated box pairs
                # while still filling fromto with the closest endpoints.  Do
                # not interpret that API quirk as contact.  Clear the buffer
                # first because pairs beyond distmax need not overwrite it.
                fromto.fill(0.0)
                raw_distance = float(
                    self.mujoco.mj_geomDistance(
                        self.model,
                        self.data,
                        geom1,
                        geom2,
                        radius,
                        fromto,
                    )
                )
                endpoint_distance = float(
                    np.linalg.norm(fromto[3:] - fromto[:3])
                )
                if abs(raw_distance) <= 1e-12 and endpoint_distance > 1e-12:
                    distance = endpoint_distance
                else:
                    # A negative raw value is authoritative penetration and
                    # must never be replaced by the unsigned endpoint norm.
                    distance = raw_distance
                # mj_geomDistance returns the search radius when no closer pair
                # was found.  Do not report that truncation boundary as a fact.
                if distance >= radius - 1e-12:
                    continue
                records.append(
                    CollisionPair(
                        geom1=self._geom_name(geom1),
                        geom2=self._geom_name(geom2),
                        body1=self.model.body(body1).name,
                        body2=self.model.body(body2).name,
                        signed_distance_m=distance,
                        normal_force_N=0.0,
                        policy_filter_reason=self._pair_filter_reason(body1, body2),
                    )
                )
        records.sort(key=lambda pair: pair.signed_distance_m)
        return DistanceAudit(radius, tuple(records))
