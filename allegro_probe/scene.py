"""MuJoCo scene shared by reference-rig and Allegro probe backends.

Both variants use a 6-DoF wrist carriage. ``poke`` and ``slide`` use an instrumented
probe; ``heft`` and ``shake`` use either a reference gripper or articulated Allegro.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple
import time
import xml.etree.ElementTree as ET

import numpy as np

from allegro_probe.geometry import quaternion_wxyz_to_matrix
from allegro_probe.models import BACKENDS, ObjectSpec, ProbeSceneSpec


DEFAULT_MENAGERIE_ROOT = Path("/home/enovo/robots/sim/mujoco_menagerie/wonik_allegro")
ALLEGRO_ACTUATORS = (
    "ffa0", "ffa1", "ffa2", "ffa3",
    "mfa0", "mfa1", "mfa2", "mfa3",
    "rfa0", "rfa1", "rfa2", "rfa3",
    "tha0", "tha1", "tha2", "tha3",
)

_ALLEGRO_OPEN = np.array([
    0.00, 0.10, 0.05, 0.05,
    0.00, 0.10, 0.05, 0.05,
    0.00, 0.10, 0.05, 0.05,
    0.45, 0.10, 0.08, 0.08,
], dtype=float)

_ALLEGRO_CYLINDER_CLOSED = np.array([
    0.10, 1.05, 0.95, 0.65,
    0.00, 1.05, 0.95, 0.65,
    -0.10, 1.05, 0.95, 0.65,
    1.05, 0.70, 0.85, 0.65,
], dtype=float)


@dataclass(frozen=True)
class SceneConfig:
    menagerie_root: Path = DEFAULT_MENAGERIE_ROOT
    backend: str = "allegro"
    candidate_spacing: float = 0.18
    palm_height: float = 0.34
    timestep: float = 0.002
    allegro_grasp_lift: float = 0.090
    short_can_place_y: float = 0.120
    full_hand_collisions: bool = False
    wrist_roll_limit_rad: float = 0.9
    wrist_tilt_limit_rad: float = 0.9
    wrist_yaw_limit_rad: float = 1.2

    def __post_init__(self) -> None:
        if self.backend not in BACKENDS:
            raise ValueError(f"backend must be one of {BACKENDS}, got {self.backend!r}")
        for name in (
            "wrist_roll_limit_rad",
            "wrist_tilt_limit_rad",
            "wrist_yaw_limit_rad",
        ):
            value = float(getattr(self, name))
            if not np.isfinite(value) or value <= 0.0 or value > np.pi:
                raise ValueError(f"{name} must be finite and in (0, pi], got {value!r}")


@dataclass(frozen=True)
class ContactSnapshot:
    """Contact facts used by primitive validity gates."""

    hand_groups: Tuple[str, ...] = ()
    hand_force_by_group_N: Tuple[Tuple[str, float], ...] = ()
    hand_object_geoms: Tuple[str, ...] = ()
    hand_contact_geoms: Tuple[str, ...] = ()
    hand_contact_bodies: Tuple[str, ...] = ()
    hand_contact_count: int = 0
    hand_normal_force_N: float = 0.0
    palm_object_contact: bool = False
    palm_object_normal_force_N: float = 0.0
    support_contact: bool = False
    support_normal_force_N: float = 0.0
    table_contact: bool = False
    table_normal_force_N: float = 0.0
    hand_table_contact: bool = False
    hand_table_normal_force_N: float = 0.0
    hand_support_contact: bool = False
    hand_support_normal_force_N: float = 0.0
    palm_table_contact: bool = False
    palm_support_contact: bool = False
    hand_other_object_contact: bool = False
    hand_other_object_normal_force_N: float = 0.0
    hand_other_object_geoms: Tuple[str, ...] = ()
    object_other_object_contact: bool = False
    object_other_object_normal_force_N: float = 0.0
    max_penetration_m: float = 0.0
    hand_max_penetration_m: float = 0.0
    support_max_penetration_m: float = 0.0
    table_max_penetration_m: float = 0.0


def _fmt(vals) -> str:
    return " ".join(f"{float(v):.6g}" for v in vals)


def _inertia_box(mass_kg: float, size: Tuple[float, float, float]) -> Tuple[float, float, float]:
    sx, sy, sz = size
    return (
        mass_kg * (sy * sy + sz * sz) / 3.0,
        mass_kg * (sx * sx + sz * sz) / 3.0,
        mass_kg * (sx * sx + sy * sy) / 3.0,
    )


def _inertia_cylinder(mass_kg: float, radius: float, half_height: float) -> Tuple[float, float, float]:
    height = 2.0 * half_height
    ix = mass_kg * (3.0 * radius * radius + height * height) / 12.0
    iz = 0.5 * mass_kg * radius * radius
    return ix, ix, iz


def _allegro_sections(
    root: Path,
    visual_only: bool = True,
    full_hand_collisions: bool = False,
) -> Tuple[str, str, str, str, str]:
    """Return default XML, asset children, palm body, actuator children, contact excludes."""

    hand_xml = root / "right_hand.xml"
    if not hand_xml.exists():
        raise FileNotFoundError(
            f"Menagerie Allegro model not found: {hand_xml}. "
            "Set SceneConfig(menagerie_root=...)."
        )

    tree = ET.parse(hand_xml)
    mj = tree.getroot()
    default = mj.find("default")
    default_xml = ET.tostring(default, encoding="unicode") if default is not None else ""

    asset = mj.find("asset")
    asset_children = ""
    if asset is not None:
        asset_children = "".join(ET.tostring(child, encoding="unicode") for child in list(asset))

    contact = mj.find("contact")
    contact_children = ""
    if contact is not None:
        contact_children = "".join(ET.tostring(child, encoding="unicode") for child in list(contact))

    palm = None
    worldbody = mj.find("worldbody")
    if worldbody is not None:
        for child in worldbody:
            if child.tag == "body" and child.attrib.get("name") == "palm":
                palm = deepcopy(child)
                break

    if palm is None:
        palm_xml = ""
    else:
        _add_allegro_tip_sites(palm)
        grasp_collision_classes = {
            "medial_collision",
            "distal_collision",
            "fingertip_collision",
            "thumb_medial_collision",
            "thumb_distal_collision",
            "thumbtip_collision",
        }
        for body in palm.iter("body"):
            body_name = body.attrib.get("name", "hand")
            for geom in body.findall("geom"):
                cls = geom.attrib.get("class", "")
                # Name collision proxies so contact gates never depend on anonymous geoms.
                if "collision" in cls and "name" not in geom.attrib:
                    geom.set("name", f"{body_name}_{cls}")
                # Object contact comes from grasping links, not broad palm/base collisions.
                if "collision" in cls:
                    if cls in grasp_collision_classes:
                        geom.set("friction", "3.5 0.06 0.004")
                        geom.set("condim", "6")
                        geom.set("priority", "3")
                    elif not full_hand_collisions:
                        geom.set("contype", "0")
                        geom.set("conaffinity", "0")
                        geom.set("mass", "0")
        if visual_only:
            _strip_joints(palm)
            palm.insert(
                0,
                ET.fromstring(
                    '<inertial pos="0 0 0" mass="0.001" '
                    'diaginertia="1e-6 1e-6 1e-6"/>'
                ),
            )
            for geom in palm.iter("geom"):
                geom.set("contype", "0")
                geom.set("conaffinity", "0")
                geom.set("mass", "0")
        palm_xml = ET.tostring(palm, encoding="unicode")

    actuator = mj.find("actuator")
    actuator_children = ""
    if actuator is not None and not visual_only:
        children = []
        for child in list(actuator):
            child = deepcopy(child)
            child.set("kp", "8")
            children.append(ET.tostring(child, encoding="unicode"))
        actuator_children = "".join(children)
    return default_xml, asset_children, palm_xml, actuator_children, contact_children


def _add_allegro_tip_sites(palm: ET.Element) -> None:
    tip_specs = {
        "ff_tip": ("ff", "0 0 0.030", "0.014"),
        "mf_tip": ("mf", "0 0 0.030", "0.014"),
        "rf_tip": ("rf", "0 0 0.030", "0.014"),
        "th_tip": ("th", "0 0 0.044", "0.015"),
    }
    for body_name, (prefix, pos, size) in tip_specs.items():
        body = _find_body(palm, body_name)
        if body is None:
            continue
        if any(child.tag == "site" and child.attrib.get("name") == f"{prefix}_tip_site"
               for child in list(body)):
            continue
        body.append(
            ET.fromstring(
                f'<site name="{prefix}_tip_site" pos="{pos}" size="{size}" '
                f'rgba="0.05 0.65 1.0 0.28"/>'
            )
        )


def _find_body(elem: ET.Element, name: str) -> Optional[ET.Element]:
    if elem.tag == "body" and elem.attrib.get("name") == name:
        return elem
    for child in elem:
        found = _find_body(child, name)
        if found is not None:
            return found
    return None


def _strip_joints(elem: ET.Element) -> None:
    for child in list(elem):
        if child.tag == "joint":
            elem.remove(child)
        else:
            _strip_joints(child)


class AllegroProbeScene:
    """Compiled MuJoCo probe scene with a primitive-oriented control/sensor API."""

    def __init__(self, task: ProbeSceneSpec, config: Optional[SceneConfig] = None):
        try:
            import mujoco  # type: ignore
        except Exception as exc:  # pragma: no cover - environment dependent
            raise RuntimeError("mujoco is required for AllegroProbeScene") from exc

        self.mujoco = mujoco
        self.task = task
        self.config = config or SceneConfig()
        self.n = task.n_candidates
        self._object_site_names: List[str] = []
        self._free_joint_names: List[Optional[str]] = []
        self.model = mujoco.MjModel.from_xml_string(self._build_xml())
        self.data = mujoco.MjData(self.model)
        self.dt = float(self.model.opt.timestep)
        self._viewer = None
        self._viewer_realtime = False
        self._grip_alpha = 0.0
        self._index()
        self._initial_object_pos = np.zeros((self.n, 3), dtype=float)
        self.reset()

    # ------------------------------------------------------------------ XML assembly
    def candidate_x(self, i: int) -> float:
        return (i - (self.n - 1) / 2.0) * self.config.candidate_spacing

    def _object_center_z(self, obj: ObjectSpec) -> float:
        z = obj.size[2] + 0.003
        if obj.family in {"mass", "fill"}:
            z += self.config.allegro_grasp_lift
        return float(z)

    def _build_xml(self) -> str:
        if self.config.backend == "allegro":
            default_xml, asset_children, palm_xml, hand_act, hand_contact = _allegro_sections(
                Path(self.config.menagerie_root),
                visual_only=False,
                full_hand_collisions=self.config.full_hand_collisions,
            )
        else:
            default_xml = asset_children = palm_xml = hand_act = hand_contact = ""

        self._object_site_names.clear()
        self._free_joint_names.clear()
        object_xml = []
        nest_xml = []
        sensors = []
        for obj in self.task.objects:
            x = self.candidate_x(obj.index)
            if obj.family in {"mass", "fill"}:
                support_top = self._object_center_z(obj) - obj.size[2]
                if support_top > 0.010:
                    # A narrow pedestal supports reset while leaving the complete waist and
                    # bottom rim accessible to a lateral grasp.  Enclosing cradle walls used
                    # by the v0 scene obstructed the hand and are intentionally absent.
                    support_half = max(0.008, min(obj.size[0], obj.size[1]) * 0.32)
                    nest_xml.append(
                        f'<geom name="obj{obj.index}_pedestal" type="cylinder" '
                        f'pos="{x:.6g} 0 {support_top / 2.0:.6g}" '
                        f'size="{support_half:.6g} {support_top / 2.0:.6g}" '
                        f'rgba="0.34 0.34 0.38 0.72" friction="1.4 0.04 0.002" condim="6"/>'
                    )

            body_xml, top_site, free_joint = self._object_xml(obj, x)
            self._object_site_names.append(top_site)
            self._free_joint_names.append(free_joint)
            object_xml.append(body_xml)
            sensors.append(f'<framepos name="obj{obj.index}_pos" objtype="site" objname="{top_site}"/>')
            sensors.append(f'<framequat name="obj{obj.index}_quat" objtype="site" objname="{top_site}"/>')

        carriage, carriage_act, carriage_sensors = self._carriage_xml(palm_xml)
        asset_block = f"""
  <asset>
    {asset_children}
    <texture name="probe_grid" type="2d" builtin="checker" rgb1="0.20 0.23 0.25"
             rgb2="0.31 0.34 0.36" width="200" height="200"/>
    <material name="probe_grid" texture="probe_grid" texrepeat="5 5" reflectance="0.08"/>
  </asset>
"""
        meshdir = Path(self.config.menagerie_root) / "assets"
        contact_xml = self._contact_xml(hand_contact, self.config.backend == "allegro")
        return f"""
<mujoco model="allegro_probe_scene">
  <compiler angle="radian" autolimits="true" meshdir="{meshdir}"/>
  <option timestep="{self.config.timestep:.6g}" integrator="implicitfast"
          cone="elliptic" impratio="10" gravity="0 0 -9.81"/>
  {default_xml}
  {asset_block}
  <worldbody>
    <light pos="0 0 1.4" dir="0 0 -1" diffuse="0.9 0.9 0.9"/>
    <geom name="floor" type="plane" size="1.4 1.4 0.1" material="probe_grid"/>
    <geom name="table" type="box" pos="0 0 -0.035" size="0.62 0.38 0.035"
          rgba="0.46 0.39 0.31 1" friction="1.2 0.04 0.002" condim="6"/>
    {''.join(nest_xml)}
    {carriage}
    {''.join(object_xml)}
  </worldbody>
  <actuator>
    {carriage_act}
    {hand_act}
  </actuator>
  <sensor>
    {carriage_sensors}
    {''.join(sensors)}
  </sensor>
  {contact_xml}
</mujoco>
""".strip()

    def _contact_xml(self, allegro_contact: str, use_allegro: bool) -> str:
        if not use_allegro:
            return ""
        probe_excludes = []
        for body in (
            "palm", "ff_base", "ff_proximal", "ff_medial", "ff_distal", "ff_tip",
            "mf_base", "mf_proximal", "mf_medial", "mf_distal", "mf_tip",
            "rf_base", "rf_proximal", "rf_medial", "rf_distal", "rf_tip",
            "th_base", "th_proximal", "th_medial", "th_distal", "th_tip",
        ):
            probe_excludes.append(f'<exclude body1="probe_force_body" body2="{body}"/>')
        return f"<contact>{allegro_contact}{''.join(probe_excludes)}</contact>"

    def _object_xml(self, obj: ObjectSpec, x: float) -> Tuple[str, str, Optional[str]]:
        rgba = _fmt(obj.rgba)
        if obj.family == "stiffness":
            sx, sy, sz = obj.size
            top_site = f"obj{obj.index}_top_site"
            damping = max(0.4, 0.004 * obj.stiffness_N_per_m)
            xml = f"""
<body name="obj{obj.index}" pos="{x:.6g} 0 0">
  <geom name="obj{obj.index}_base" type="box" pos="0 0 0.006"
        size="{sx:.6g} {sy:.6g} 0.006" rgba="{rgba}"
        contype="0" conaffinity="0"/>
  <body name="obj{obj.index}_top" pos="0 0 {sz + 0.012:.6g}">
    <joint name="obj{obj.index}_compress" type="slide" axis="0 0 1"
           range="-0.012 0" stiffness="{obj.stiffness_N_per_m:.6g}"
           damping="{damping:.6g}" springref="0"/>
    <inertial pos="0 0 0" mass="{obj.mass_kg:.6g}"
              diaginertia="{_fmt(_inertia_box(obj.mass_kg, obj.size))}"/>
    <geom name="obj{obj.index}_geom" type="box" size="{sx:.6g} {sy:.6g} {sz:.6g}"
          rgba="{rgba}" friction="1.0 0.02 0.001" condim="6" priority="2"
          solref="0.004 1" solimp="0.95 0.995 0.001"/>
    <site name="{top_site}" pos="0 0 {sz:.6g}" size="0.004"/>
  </body>
</body>
"""
            return xml, top_site, None

        sx, sy, sz = obj.size
        top_site = f"obj{obj.index}_top_site"
        free = obj.family in {"mass", "fill"}
        free_joint = f"obj{obj.index}_free" if free else None
        joint_xml = f'<freejoint name="{free_joint}"/>' if free else ""
        pos_z = self._object_center_z(obj)
        mass_shell = max(obj.mass_kg - obj.liquid_mass_kg, 1e-4)
        if obj.shape in {"short_can", "opaque_cup"}:
            inertia = _inertia_cylinder(mass_shell, sx, sz)
            waist_radius = sx * (0.78 if obj.shape == "short_can" else 0.82)
            lip_half_height = max(0.0045, sz * 0.16)
            lip_z = max(0.0, sz - lip_half_height)
            waist_half_height = max(0.004, sz - 2.0 * lip_half_height)
            geom_xml = (
                f'<geom name="obj{obj.index}_geom" type="cylinder" size="{waist_radius:.6g} {waist_half_height:.6g}" '
                f'rgba="{rgba}" friction="{obj.friction_mu:.6g} 0.04 0.003" condim="6" priority="2"/>'
                f'<geom name="obj{obj.index}_top_lip" type="cylinder" pos="0 0 {lip_z:.6g}" '
                f'size="{sx:.6g} {lip_half_height:.6g}" rgba="{rgba}" '
                f'friction="{obj.friction_mu:.6g} 0.04 0.003" condim="6" priority="2"/>'
                f'<geom name="obj{obj.index}_bottom_lip" type="cylinder" pos="0 0 {-lip_z:.6g}" '
                f'size="{sx:.6g} {lip_half_height:.6g}" rgba="{rgba}" '
                f'friction="{obj.friction_mu:.6g} 0.04 0.003" condim="6" priority="2"/>'
            )
        else:
            inertia = _inertia_box(mass_shell, obj.size)
            geom_xml = (
                f'<geom name="obj{obj.index}_geom" type="box" size="{sx:.6g} {sy:.6g} {sz:.6g}" '
                f'rgba="{rgba}" friction="{obj.friction_mu:.6g} 0.02 0.001" condim="6" priority="2"/>'
            )

        liquid_xml = ""
        if obj.family == "fill" and obj.liquid_mass_kg > 1e-5:
            r = max(min(sx, sy) * 0.32, 0.006)
            rng = max(obj.slosh_range_m, 0.001)
            liquid_xml = f"""
  <body name="obj{obj.index}_liquid" pos="0 0 {-0.30 * sz:.6g}">
    <joint name="obj{obj.index}_slosh_x" type="slide" axis="1 0 0"
           range="{-rng:.6g} {rng:.6g}" damping="0.8" stiffness="4.0"/>
    <joint name="obj{obj.index}_slosh_y" type="slide" axis="0 1 0"
           range="{-rng:.6g} {rng:.6g}" damping="0.8" stiffness="4.0"/>
    <geom name="obj{obj.index}_hidden_liquid" type="sphere" size="{r:.6g}"
          mass="{obj.liquid_mass_kg:.6g}" rgba="0.1 0.2 0.8 0"
          contype="0" conaffinity="0"/>
  </body>
"""

        xml = f"""
<body name="obj{obj.index}" pos="{x:.6g} 0 {pos_z:.6g}">
  {joint_xml}
  <inertial pos="0 0 0" mass="{mass_shell:.6g}" diaginertia="{_fmt(inertia)}"/>
  {geom_xml}
  <site name="{top_site}" pos="0 0 {sz:.6g}" size="0.004"/>
  {liquid_xml}
</body>
"""
        return xml, top_site, free_joint

    def _carriage_xml(self, palm_xml: str) -> Tuple[str, str, str]:
        cfg = self.config
        inert = '<inertial pos="0 0 0" mass="0.08" diaginertia="8e-5 8e-5 8e-5"/>'
        probe_collision = (
            'contype="1" conaffinity="1"'
            if self.task.family in {"stiffness", "material"}
            else 'contype="0" conaffinity="0"'
        )
        body = [
            f'<body name="wrist_carriage" pos="0 0 {cfg.palm_height:.6g}">',
            '<joint name="wx" type="slide" axis="1 0 0" range="-0.55 0.55" damping="60" armature="0.1"/>',
            '<joint name="wy" type="slide" axis="0 1 0" range="-0.35 0.35" damping="60" armature="0.1"/>',
            '<joint name="wz" type="slide" axis="0 0 1" range="-0.55 0.14" damping="60" armature="0.1"/>',
            f'<joint name="wr" type="hinge" axis="1 0 0" range="{-cfg.wrist_roll_limit_rad:.12g} {cfg.wrist_roll_limit_rad:.12g}" damping="6" armature="0.05"/>',
            f'<joint name="wt" type="hinge" axis="0 1 0" range="{-cfg.wrist_tilt_limit_rad:.12g} {cfg.wrist_tilt_limit_rad:.12g}" damping="6" armature="0.05"/>',
            f'<joint name="wyaw" type="hinge" axis="0 0 1" range="{-cfg.wrist_yaw_limit_rad:.12g} {cfg.wrist_yaw_limit_rad:.12g}" damping="4" armature="0.03"/>',
            inert,
            '<body name="wrist_ft_body">',
            '<site name="wrist_ft_site" pos="0 0 0" size="0.006" rgba="0.2 0.9 0.2 0.3"/>',
            '<site name="wrist_pose_site" pos="0 0 0" size="0.003" rgba="0 0 0 0"/>',
            palm_xml if cfg.backend == "allegro" else "",
            '<body name="probe_mount" pos="0 0 0">',
            '<joint name="wp" type="slide" axis="0 0 -1" range="0 0.18" damping="8" armature="0.02"/>',
            inert,
            '<body name="probe_force_body">',
            '<site name="probe_force_site" pos="0 0 0" size="0.004"/>',
            '<geom name="probe_tip_geom" type="capsule" fromto="0 0 0.060 0 0 0" size="0.005" '
            f'rgba="0.9 0.15 0.10 1" friction="1.4 0.02 0.001" condim="6" {probe_collision}/>',
            '<site name="probe_tip_site" pos="0 0 0" size="0.007" rgba="0.9 0.15 0.10 0.25"/>',
            '</body>',
            '</body>',
        ]

        if cfg.backend == "reference":
            jaw_collision = (
                'contype="1" conaffinity="1"'
                if self.task.family in {"mass", "fill"}
                else 'contype="0" conaffinity="0"'
            )
            body.extend([
                '<body name="ref_left_jaw" pos="0 -0.060 0">',
                '<joint name="ref_left_close" type="slide" axis="0 1 0" range="0 0.040" damping="4"/>',
                '<geom name="ref_left_jaw_geom" type="box" pos="0 0 0" size="0.012 0.008 0.028" '
                f'friction="4.0 0.04 0.003" condim="6" rgba="0.2 0.55 0.9 1" {jaw_collision}/>',
                '<geom name="ref_left_hook_geom" type="box" pos="0 0.010 -0.039" '
                f'size="0.012 0.006 0.003" friction="2.0 0.03 0.002" condim="6" '
                f'rgba="0.15 0.45 0.8 1" {jaw_collision}/>',
                '<site name="ref_left_touch_site" pos="0 0.008 0" size="0.014 0.010 0.030" type="box"/>',
                '</body>',
                '<body name="ref_right_jaw" pos="0 0.060 0">',
                '<joint name="ref_right_close" type="slide" axis="0 -1 0" range="0 0.040" damping="4"/>',
                '<geom name="ref_right_jaw_geom" type="box" pos="0 0 0" size="0.012 0.008 0.028" '
                f'friction="4.0 0.04 0.003" condim="6" rgba="0.2 0.55 0.9 1" {jaw_collision}/>',
                '<geom name="ref_right_hook_geom" type="box" pos="0 -0.010 -0.039" '
                f'size="0.012 0.006 0.003" friction="2.0 0.03 0.002" condim="6" '
                f'rgba="0.15 0.45 0.8 1" {jaw_collision}/>',
                '<site name="ref_right_touch_site" pos="0 -0.008 0" size="0.014 0.010 0.030" type="box"/>',
                '</body>',
            ])

        body.extend(["</body>", "</body>"])

        act = [
            '<position name="act_wx" joint="wx" kp="650" ctrlrange="-0.55 0.55"/>',
            '<position name="act_wy" joint="wy" kp="650" ctrlrange="-0.35 0.35"/>',
            '<position name="act_wz" joint="wz" kp="900" ctrlrange="-0.55 0.14"/>',
            f'<position name="act_wr" joint="wr" kp="120" ctrlrange="{-cfg.wrist_roll_limit_rad:.12g} {cfg.wrist_roll_limit_rad:.12g}"/>',
            f'<position name="act_wt" joint="wt" kp="120" ctrlrange="{-cfg.wrist_tilt_limit_rad:.12g} {cfg.wrist_tilt_limit_rad:.12g}"/>',
            f'<position name="act_wyaw" joint="wyaw" kp="80" ctrlrange="{-cfg.wrist_yaw_limit_rad:.12g} {cfg.wrist_yaw_limit_rad:.12g}"/>',
            '<position name="act_wp" joint="wp" kp="180" ctrlrange="0 0.18"/>',
        ]
        if cfg.backend == "reference":
            act.extend([
                '<position name="act_ref_left" joint="ref_left_close" kp="220" '
                'ctrlrange="0 0.040" forcerange="-45 45"/>',
                '<position name="act_ref_right" joint="ref_right_close" kp="220" '
                'ctrlrange="0 0.040" forcerange="-45 45"/>',
            ])
        sensors = [
            '<touch name="probe_touch" site="probe_tip_site"/>',
            '<force name="probe_force" site="probe_force_site"/>',
            '<framepos name="probe_framepos" objtype="site" objname="probe_tip_site"/>',
            '<force name="wrist_force" site="wrist_ft_site"/>',
            '<torque name="wrist_torque" site="wrist_ft_site"/>',
            '<framepos name="wrist_framepos" objtype="site" objname="wrist_pose_site"/>',
            '<framequat name="wrist_framequat" objtype="site" objname="wrist_pose_site"/>',
        ]
        for j in ("wx", "wy", "wz", "wr", "wt", "wyaw", "wp"):
            sensors.append(f'<jointpos name="{j}_pos" joint="{j}"/>')
            sensors.append(f'<jointvel name="{j}_vel" joint="{j}"/>')
        if cfg.backend == "allegro":
            for prefix in ("ff", "mf", "rf", "th"):
                sensors.append(f'<touch name="{prefix}_tip_touch" site="{prefix}_tip_site"/>')
                sensors.append(f'<framepos name="{prefix}_tip_pos" objtype="site" objname="{prefix}_tip_site"/>')
            for aname in ALLEGRO_ACTUATORS:
                sensors.append(f'<actuatorfrc name="{aname}_frc" actuator="{aname}"/>')
                joint = aname.replace("a", "j")
                sensors.append(
                    f'<jointactuatorfrc name="{joint}_actuatorfrc" joint="{joint}"/>'
                )
        else:
            sensors.extend([
                '<touch name="ref_left_touch" site="ref_left_touch_site"/>',
                '<touch name="ref_right_touch" site="ref_right_touch_site"/>',
                '<jointactuatorfrc name="ref_left_actuatorfrc" joint="ref_left_close"/>',
                '<jointactuatorfrc name="ref_right_actuatorfrc" joint="ref_right_close"/>',
            ])
        return "".join(body), "".join(act), "".join(sensors)

    # ------------------------------------------------------------------ indexing/reset
    def _index(self) -> None:
        mj = self.mujoco
        self.sensor_index: Dict[str, Tuple[int, int]] = {}
        for sid in range(self.model.nsensor):
            name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_SENSOR, sid)
            self.sensor_index[name] = (int(self.model.sensor_adr[sid]), int(self.model.sensor_dim[sid]))
        self.geom: Dict[str, int] = {}
        self.geom_body_name: Dict[int, str] = {}
        for gid in range(self.model.ngeom):
            name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_GEOM, gid)
            if name:
                self.geom[name] = gid
            bid = int(self.model.geom_bodyid[gid])
            self.geom_body_name[gid] = (
                mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_BODY, bid) or ""
            )
        probe_gid = self.geom.get("probe_tip_geom")
        if probe_gid is not None:
            self._probe_contype = int(self.model.geom_contype[probe_gid])
            self._probe_conaffinity = int(self.model.geom_conaffinity[probe_gid])
        else:
            self._probe_contype = 0
            self._probe_conaffinity = 0
        self.act: Dict[str, int] = {}
        for aid in range(self.model.nu):
            name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_ACTUATOR, aid)
            self.act[name] = aid
        self.joint_qadr: Dict[str, int] = {}
        for jid in range(self.model.njnt):
            name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_JOINT, jid)
            self.joint_qadr[name] = int(self.model.jnt_qposadr[jid])

    @property
    def sensor_names(self) -> List[str]:
        return sorted(self.sensor_index)

    def full_hand_collisions_compiled(self) -> bool:
        """Return whether every palm/base/proximal collision proxy is active."""

        if self.config.backend != "allegro":
            return False
        required = (
            "palm_palm_collision",
            "ff_base_base_collision",
            "ff_proximal_proximal_collision",
            "mf_base_base_collision",
            "mf_proximal_proximal_collision",
            "rf_base_base_collision",
            "rf_proximal_proximal_collision",
            "th_base_thumb_base_collision",
            "th_proximal_thumb_proximal_collision",
        )
        for name in required:
            gid = self.geom.get(name)
            if gid is None:
                return False
            if (
                int(self.model.geom_contype[gid]) == 0
                or int(self.model.geom_conaffinity[gid]) == 0
            ):
                return False
        return True

    def reset(self) -> None:
        mj = self.mujoco
        mj.mj_resetData(self.model, self.data)
        for i, free_joint in enumerate(self._free_joint_names):
            if not free_joint:
                continue
            qadr = self.joint_qadr[free_joint]
            obj = self.task.objects[i]
            self.data.qpos[qadr:qadr + 3] = [self.candidate_x(i), 0.0, self._object_center_z(obj)]
            self.data.qpos[qadr + 3:qadr + 7] = [1.0, 0.0, 0.0, 0.0]

        self.command(
            x=0.0,
            y=0.0,
            z=0.10,
            roll=0.0,
            tilt=0.0,
            yaw=0.0,
            probe=0.0,
            grip=0.0,
        )
        self._set_hand_neutral()
        mj.mj_forward(self.model, self.data)
        self.step(150)
        self._initial_object_pos = np.array([self.object_pos(i) for i in range(self.n)])

    def _set_hand_neutral(self) -> None:
        self.command_grip(0.0)

    def set_probe_collision(self, enabled: bool) -> None:
        """Compatibility shim.

        Probe collision is fixed when the model is compiled: enabled for poke/slide
        scenes and disabled for heft/shake scenes.  Runtime collision spoofing is
        deliberately rejected.
        """
        gid = self.geom.get("probe_tip_geom")
        if gid is None:
            return
        expected = self.task.family in {"stiffness", "material"}
        if bool(enabled) != expected:
            raise RuntimeError(
                "probe collision is fixed per scene; construct the matching family/backend"
            )

    def attach_viewer(self, viewer, realtime: bool = True) -> None:
        self._viewer = viewer
        self._viewer_realtime = bool(realtime)
        self._viewer.sync()

    def detach_viewer(self) -> None:
        self._viewer = None
        self._viewer_realtime = False

    # ------------------------------------------------------------------ control/sensors
    def command(
        self,
        x: Optional[float] = None,
        y: Optional[float] = None,
        z: Optional[float] = None,
        roll: Optional[float] = None,
        tilt: Optional[float] = None,
        yaw: Optional[float] = None,
        probe: Optional[float] = None,
        grip: Optional[float] = None,
    ) -> None:
        if x is not None:
            self.data.ctrl[self.act["act_wx"]] = float(x)
        if y is not None:
            self.data.ctrl[self.act["act_wy"]] = float(y)
        if z is not None:
            self.data.ctrl[self.act["act_wz"]] = float(z)
        if roll is not None:
            self.data.ctrl[self.act["act_wr"]] = float(roll)
        if tilt is not None:
            self.data.ctrl[self.act["act_wt"]] = float(tilt)
        if yaw is not None:
            self.data.ctrl[self.act["act_wyaw"]] = float(yaw)
        if probe is not None:
            self.data.ctrl[self.act["act_wp"]] = float(probe)
        if grip is not None:
            self.command_grip(float(np.clip(grip, 0.0, 1.0)))

    def command_grip(self, alpha: float) -> None:
        a = float(np.clip(alpha, 0.0, 1.0))
        self._grip_alpha = a
        if self.config.backend == "allegro":
            self.command_allegro_grip(a)
            return
        target = 0.032 * a
        self.data.ctrl[self.act["act_ref_left"]] = target
        self.data.ctrl[self.act["act_ref_right"]] = target

    def command_allegro_grip(self, alpha: float) -> None:
        if "ffa0" not in self.act:
            return
        self.command_allegro_joints(self.allegro_grip_pose(alpha))

    @staticmethod
    def allegro_grip_pose(alpha: float) -> np.ndarray:
        """Return the legacy cylinder-synergy pose at one closure progress."""

        a = float(np.clip(alpha, 0.0, 1.0))
        return (1.0 - a) * _ALLEGRO_OPEN + a * _ALLEGRO_CYLINDER_CLOSED

    def command_allegro_joints(self, targets: np.ndarray) -> None:
        """Command all 16 Allegro position actuators explicitly.

        The public manipulation path uses object-specific 16-DoF hand templates.
        ``command_grip`` remains as the one-dimensional compatibility path used by
        the probe primitives.
        """

        if self.config.backend != "allegro":
            raise RuntimeError("16-DoF Allegro commands require backend='allegro'")
        pose = np.asarray(targets, dtype=float)
        if pose.shape != (len(ALLEGRO_ACTUATORS),):
            raise ValueError(
                f"expected {len(ALLEGRO_ACTUATORS)} Allegro joint targets, "
                f"got shape {pose.shape}"
            )
        for idx, name in enumerate(ALLEGRO_ACTUATORS):
            aid = self.act.get(name)
            if aid is None:
                continue
            val = float(pose[idx])
            if self.model.actuator_ctrllimited[aid]:
                lo, hi = self.model.actuator_ctrlrange[aid]
                val = float(np.clip(val, lo, hi))
            self.data.ctrl[aid] = val

    def allegro_joint_targets(self) -> np.ndarray:
        if self.config.backend != "allegro":
            return np.zeros(0, dtype=float)
        return np.asarray(
            [self.data.ctrl[self.act[name]] for name in ALLEGRO_ACTUATORS],
            dtype=float,
        )

    def allegro_joint_positions(self) -> np.ndarray:
        if self.config.backend != "allegro":
            return np.zeros(0, dtype=float)
        return np.asarray(
            [
                self.data.qpos[self.joint_qadr[name.replace("a", "j")]]
                for name in ALLEGRO_ACTUATORS
            ],
            dtype=float,
        )

    def set_allegro_position_kp(self, kp: float) -> None:
        """Set the simulated Allegro position-servo stiffness for guarded phases."""

        if self.config.backend != "allegro":
            raise RuntimeError("Allegro gain scheduling requires backend='allegro'")
        value = float(kp)
        if not np.isfinite(value) or value <= 0.0:
            raise ValueError(f"Allegro kp must be positive and finite, got {kp!r}")
        for name in ALLEGRO_ACTUATORS:
            aid = self.act[name]
            self.model.actuator_gainprm[aid, 0] = value
            self.model.actuator_biasprm[aid, 1] = -value

    def step(self, n: int = 1) -> None:
        for _ in range(int(n)):
            self.mujoco.mj_step(self.model, self.data)
            if self._viewer is not None and self._viewer.is_running():
                self._viewer.sync()
                if self._viewer_realtime:
                    time.sleep(self.dt)

    def sensor(self, name: str) -> np.ndarray:
        adr, dim = self.sensor_index[name]
        return np.asarray(self.data.sensordata[adr:adr + dim], dtype=float)

    def read_sensors(self) -> Dict[str, np.ndarray | float]:
        out: Dict[str, np.ndarray | float] = {}
        for name in self.sensor_index:
            val = self.sensor(name)
            out[name] = float(val[0]) if val.size == 1 else val
        return out

    def object_pos(self, i: int) -> np.ndarray:
        return self.sensor(f"obj{i}_pos")

    def object_quat(self, i: int) -> np.ndarray:
        return self.sensor(f"obj{i}_quat")

    def object_center_pos(self, i: int) -> np.ndarray:
        """Return the geometry-center position, including for tilted objects.

        ``object_pos`` is retained for compatibility and returns the top-site
        position.  New pose-conditioned manipulation must use this method.
        """

        local_top = np.asarray([0.0, 0.0, self.task.objects[i].size[2]], dtype=float)
        world_top_offset = quaternion_wxyz_to_matrix(self.object_quat(i)) @ local_top
        return self.object_pos(i) - world_top_offset

    def set_object_pose(
        self,
        i: int,
        *,
        center_position_m: Tuple[float, float, float],
        quaternion_wxyz: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
        zero_velocity: bool = True,
        record_initial: bool = False,
    ) -> None:
        """Place one free object at an explicit geometry-center world pose."""

        index = int(i)
        if index < 0 or index >= self.n:
            raise IndexError(f"object index {index} outside [0, {self.n})")
        free_joint = self._free_joint_names[index]
        if not free_joint:
            raise ValueError(f"object {index} does not have a free joint")
        position = np.asarray(center_position_m, dtype=float)
        quaternion = np.asarray(quaternion_wxyz, dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("center_position_m must be a finite 3-vector")
        if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
            raise ValueError("quaternion_wxyz must be a finite 4-vector")
        norm = float(np.linalg.norm(quaternion))
        if norm <= 1e-12:
            raise ValueError("quaternion_wxyz must have non-zero norm")
        quaternion /= norm

        qadr = self.joint_qadr[free_joint]
        self.data.qpos[qadr:qadr + 3] = position
        self.data.qpos[qadr + 3:qadr + 7] = quaternion
        if zero_velocity:
            jid = self.mujoco.mj_name2id(
                self.model, self.mujoco.mjtObj.mjOBJ_JOINT, free_joint
            )
            dadr = int(self.model.jnt_dofadr[jid])
            self.data.qvel[dadr:dadr + 6] = 0.0

        self.mujoco.mj_forward(self.model, self.data)
        if record_initial:
            self._initial_object_pos[index] = self.object_pos(index)

    def object_displacement(self, i: int) -> float:
        return float(np.linalg.norm(self.object_pos(i) - self._initial_object_pos[i]))

    def object_top_z(self, i: int) -> float:
        return float(self.object_pos(i)[2])

    def object_mid_z(self, i: int) -> float:
        return float(self.object_top_z(i) - self.task.objects[i].size[2])

    def object_lifted(self, i: int, min_height: float = 0.015) -> bool:
        return bool(self.object_pos(i)[2] > self._initial_object_pos[i][2] + min_height)

    def probe_tip_pos(self) -> np.ndarray:
        return self.sensor("probe_framepos")

    def probe_touch(self) -> float:
        return float(self.sensor("probe_touch")[0])

    def probe_force_vec(self) -> np.ndarray:
        return self.sensor("probe_force")

    def wrist_force_vec(self) -> np.ndarray:
        return self.sensor("wrist_force")

    def wrist_torque_vec(self) -> np.ndarray:
        return self.sensor("wrist_torque")

    def wrist_pos(self) -> np.ndarray:
        return self.sensor("wrist_framepos")

    def wrist_quat(self) -> np.ndarray:
        return self.sensor("wrist_framequat")

    def finger_touch_total(self) -> float:
        if self.config.backend == "reference":
            return float(
                self.sensor("ref_left_touch")[0] + self.sensor("ref_right_touch")[0]
            )
        return float(
            sum(
                float(self.sensor(f"{prefix}_tip_touch")[0])
                for prefix in ("ff", "mf", "rf", "th")
            )
        )

    def fingertip_positions(self) -> Dict[str, np.ndarray]:
        if self.config.backend == "reference":
            return {}
        return {
            prefix: self.sensor(f"{prefix}_tip_pos")
            for prefix in ("ff", "mf", "rf", "th")
        }

    def wz_for_tip_z(self, z_world: float, probe_extension: float = 0.0) -> float:
        return float(np.clip(z_world - self.config.palm_height + probe_extension, -0.55, 0.14))

    def relative_object_position(self, i: int) -> np.ndarray:
        return self.object_pos(i) - self.wrist_pos()

    def contact_snapshot(self, i: int) -> ContactSnapshot:
        """Summarize actual MuJoCo contacts involving one object.

        Contacts are classified by geom/body identity rather than touch-site coverage,
        because an Allegro wrap grasp may use medial and distal links legitimately.
        """

        object_prefix = f"obj{i}_"
        support_name = f"obj{i}_pedestal"
        hand_groups: Set[str] = set()
        hand_force_by_group: Dict[str, float] = {}
        hand_object_geoms: Set[str] = set()
        hand_contact_geoms: Set[str] = set()
        hand_contact_bodies: Set[str] = set()
        hand_count = 0
        hand_force = 0.0
        palm_object_contact = False
        palm_object_force = 0.0
        support_contact = False
        support_force = 0.0
        table_contact = False
        table_force = 0.0
        hand_table_contact = False
        hand_table_force = 0.0
        hand_support_contact = False
        hand_support_force = 0.0
        palm_table_contact = False
        palm_support_contact = False
        hand_other_object_contact = False
        hand_other_object_force = 0.0
        hand_other_object_geoms: Set[str] = set()
        object_other_object_contact = False
        object_other_object_force = 0.0
        max_penetration = 0.0
        hand_max_penetration = 0.0
        support_max_penetration = 0.0
        table_max_penetration = 0.0

        def geom_name(gid: int) -> str:
            return (
                self.mujoco.mj_id2name(
                    self.model, self.mujoco.mjtObj.mjOBJ_GEOM, int(gid)
                )
                or ""
            )

        def is_object_geom(name: str) -> bool:
            return (
                name.startswith(object_prefix)
                and name != support_name
                and "pedestal" not in name
            )

        def is_any_object_geom(name: str) -> bool:
            return (
                name.startswith("obj")
                and "pedestal" not in name
                and "hidden_liquid" not in name
            )

        def is_palm(body_name: str, name: str) -> bool:
            return body_name == "palm" or name.startswith("palm_")

        for ci in range(int(self.data.ncon)):
            contact = self.data.contact[ci]
            g1, g2 = int(contact.geom1), int(contact.geom2)
            n1, n2 = geom_name(g1), geom_name(g2)
            wrench = np.zeros(6, dtype=float)
            self.mujoco.mj_contactForce(self.model, self.data, ci, wrench)
            normal_force = max(float(wrench[0]), 0.0)
            penetration = max(-float(contact.dist), 0.0)

            b1 = self.geom_body_name.get(g1, "")
            b2 = self.geom_body_name.get(g2, "")
            hand1 = self._hand_contact_group(b1, n1)
            hand2 = self._hand_contact_group(b2, n2)
            palm1 = is_palm(b1, n1)
            palm2 = is_palm(b2, n2)
            hand_other_pair = (
                (hand1 is not None or palm1)
                and is_any_object_geom(n2)
                and not is_object_geom(n2)
            ) or (
                (hand2 is not None or palm2)
                and is_any_object_geom(n1)
                and not is_object_geom(n1)
            )
            if hand_other_pair:
                hand_other_object_contact = (
                    hand_other_object_contact or normal_force > 1e-4
                )
                hand_other_object_force += normal_force
                hand_other_object_geoms.add(
                    n2 if is_any_object_geom(n2) else n1
                )
            object_other_pair = (
                is_object_geom(n1)
                and is_any_object_geom(n2)
                and not is_object_geom(n2)
            ) or (
                is_object_geom(n2)
                and is_any_object_geom(n1)
                and not is_object_geom(n1)
            )
            if object_other_pair:
                object_other_object_contact = (
                    object_other_object_contact or normal_force > 1e-4
                )
                object_other_object_force += normal_force
            hand_table_pair = (
                n1 in {"table", "floor"} and (hand2 is not None or palm2)
            ) or (n2 in {"table", "floor"} and (hand1 is not None or palm1))
            if hand_table_pair:
                hand_table_contact = hand_table_contact or normal_force > 1e-4
                hand_table_force += normal_force
                palm_table_contact = palm_table_contact or (
                    normal_force > 1e-4 and (palm1 or palm2)
                )
            hand_support_pair = (
                "pedestal" in n1 and (hand2 is not None or palm2)
            ) or ("pedestal" in n2 and (hand1 is not None or palm1))
            if hand_support_pair:
                hand_support_contact = hand_support_contact or normal_force > 1e-4
                hand_support_force += normal_force
                palm_support_contact = palm_support_contact or (
                    normal_force > 1e-4 and (palm1 or palm2)
                )

            if is_object_geom(n1):
                other_gid, other_name = g2, n2
            elif is_object_geom(n2):
                other_gid, other_name = g1, n1
            else:
                continue

            max_penetration = max(max_penetration, penetration)

            if other_name == support_name:
                support_contact = support_contact or normal_force > 1e-4
                support_force += normal_force
                support_max_penetration = max(support_max_penetration, penetration)
                continue
            if other_name in {"table", "floor"}:
                table_contact = table_contact or normal_force > 1e-4
                table_force += normal_force
                table_max_penetration = max(table_max_penetration, penetration)
                continue

            body_name = self.geom_body_name.get(other_gid, "")
            group = self._hand_contact_group(body_name, other_name)
            if group is not None:
                hand_object_geoms.add(n1 if is_object_geom(n1) else n2)
                hand_contact_geoms.add(other_name)
                hand_contact_bodies.add(body_name)
                if normal_force > 1e-4:
                    hand_groups.add(group)
                hand_force_by_group[group] = (
                    hand_force_by_group.get(group, 0.0) + normal_force
                )
                hand_max_penetration = max(hand_max_penetration, penetration)
                hand_count += 1
                hand_force += normal_force
            elif is_palm(body_name, other_name):
                palm_object_contact = palm_object_contact or normal_force > 1e-4
                palm_object_force += normal_force
                hand_max_penetration = max(hand_max_penetration, penetration)

        return ContactSnapshot(
            hand_groups=tuple(sorted(hand_groups)),
            hand_force_by_group_N=tuple(sorted(hand_force_by_group.items())),
            hand_object_geoms=tuple(sorted(hand_object_geoms)),
            hand_contact_geoms=tuple(sorted(hand_contact_geoms)),
            hand_contact_bodies=tuple(sorted(hand_contact_bodies)),
            hand_contact_count=hand_count,
            hand_normal_force_N=hand_force,
            palm_object_contact=palm_object_contact,
            palm_object_normal_force_N=palm_object_force,
            support_contact=support_contact,
            support_normal_force_N=support_force,
            table_contact=table_contact,
            table_normal_force_N=table_force,
            hand_table_contact=hand_table_contact,
            hand_table_normal_force_N=hand_table_force,
            hand_support_contact=hand_support_contact,
            hand_support_normal_force_N=hand_support_force,
            palm_table_contact=palm_table_contact,
            palm_support_contact=palm_support_contact,
            hand_other_object_contact=hand_other_object_contact,
            hand_other_object_normal_force_N=hand_other_object_force,
            hand_other_object_geoms=tuple(sorted(hand_other_object_geoms)),
            object_other_object_contact=object_other_object_contact,
            object_other_object_normal_force_N=object_other_object_force,
            max_penetration_m=max_penetration,
            hand_max_penetration_m=hand_max_penetration,
            support_max_penetration_m=support_max_penetration,
            table_max_penetration_m=table_max_penetration,
        )

    def _hand_contact_group(self, body_name: str, geom_name: str) -> Optional[str]:
        if self.config.backend == "reference":
            if body_name == "ref_left_jaw" or geom_name.startswith("ref_left"):
                return "left"
            if body_name == "ref_right_jaw" or geom_name.startswith("ref_right"):
                return "right"
            return None
        for prefix in ("ff", "mf", "rf", "th"):
            if body_name.startswith(prefix) or geom_name.startswith(prefix):
                return prefix
        return None

    def has_opposing_grasp(self, snapshot: ContactSnapshot) -> bool:
        groups = set(snapshot.hand_groups)
        if self.config.backend == "reference":
            return {"left", "right"}.issubset(groups)
        return "th" in groups and bool(groups.intersection({"ff", "mf", "rf"}))
