# go2stand.py
# Control Unitree Go2 in MuJoCo: stand, then optional trot (sim-only, no DDS).
# Run: python -m unitree_sim.sim.go2stand --mode stand|trot [--viz]

import argparse
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import mujoco as mj

try:
    # Optional viewer (only used with --viz)
    from mujoco import viewer
    HAS_VIEWER = True
except Exception:
    HAS_VIEWER = False


# ---------------------------
# Helpers to load the Go2 XML
# ---------------------------
def _resolve_go2_xml() -> Path:
    """
    Use the same XML path that test_go2.py uses.
    """
    return Path.home() / "Unitree_Module" / "Unitree_Sim" / "unitree_sim" / "robots" / "go2" / "scene.xml"


def _reset_to_keyframe(model: mj.MjModel, data: mj.MjData, key_name: str = "home") -> None:
    """Reset simulation state to a named keyframe and refresh derived quantities."""
    key_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_KEY, key_name)
    if key_id < 0:
        raise RuntimeError(f"Keyframe '{key_name}' not found in model")
    mj.mj_resetDataKeyframe(model, data, key_id)
    mj.mj_forward(model, data)


# ---------------------------------------
# SDK-like "LowCmd" shim for MuJoCo sim
# ---------------------------------------
@dataclass
class JointCmd:
    q: float = 0.0           # desired position
    dq: float = 0.0          # desired velocity
    kp: float = 0.0          # position gain
    kd: float = 0.0          # velocity gain
    tau_ff: float = 0.0      # feedforward torque


class LowCmdShim:
    """
    Minimal SDK-like command object for 12 joints.
    Access by name, e.g., cmd['FR_hip'].q = 0.1
    """
    def __init__(self, joint_names: List[str]):
        self._cmd: Dict[str, JointCmd] = {jn: JointCmd() for jn in joint_names}

    def __getitem__(self, jn: str) -> JointCmd:
        return self._cmd[jn]

    def items(self):
        return self._cmd.items()


# ---------------------------------------
# Joint name mapping & default poses
# ---------------------------------------
def default_joint_order() -> List[str]:
    """
    A common naming for Unitree-style quadrupeds in MuJoCo.
    If your XML differs, print mj_name2id to map properly.
    """
    # legs: FR, FL, RR, RL ; joints: hip(ab/ad), thigh(pitch), calf(knee)
    return [
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    ]


def stand_pose() -> Dict[str, float]:
    """
    Reasonable stand for Unitree-like legs (radians).
    hip(ab/ad)=0, thigh(pitch)=~0.9, calf(knee)=~-1.8 (A1/Go2-like).
    Tune to your XML.
    """
    angles = {
        "FR_hip_joint": 0.0, "FR_thigh_joint": 0.9, "FR_calf_joint": -1.8,
        "FL_hip_joint": 0.0, "FL_thigh_joint": 0.9, "FL_calf_joint": -1.8,
        "RR_hip_joint": 0.0, "RR_thigh_joint": 0.9, "RR_calf_joint": -1.8,
        "RL_hip_joint": 0.0, "RL_thigh_joint": 0.9, "RL_calf_joint": -1.8,
    }
    return angles


# ---------------------------------------
# Controller core
# ---------------------------------------
class Go2Controller:
    def __init__(self, model: mj.MjModel, data: mj.MjData, joint_names: List[str]):
        self.model = model
        self.data = data
        self.joint_names = joint_names
        self.jnt_id = [mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, jn) for jn in joint_names]
        self.dof_id = [self.model.jnt_dofadr[jid] for jid in self.jnt_id]
        # Map each joint to its actuator index so we write the correct ctrl slot
        self.act_id: List[int] = []
        for jid in self.jnt_id:
            found = None
            for aid in range(self.model.nu):
                if self.model.actuator_trnid[aid, 0] == jid:
                    found = aid
                    break
            if found is None:
                raise RuntimeError(
                    "Each commanded joint must have a dedicated actuator; none found for joint "
                    f"id {jid}."
                )
            self.act_id.append(found)
        self.name_to_idx = {jn: i for i, jn in enumerate(joint_names)}

        self.cmd = LowCmdShim(joint_names)
        self.desired_q = self._q()

        # Default gains (conservative; increase as needed)
        for jn in joint_names:
            if jn.endswith("hip_joint"):
                kp, kd = 60.0, 3.0
            elif jn.endswith("thigh_joint"):
                kp, kd = 140.0, 7.0
            else:
                kp, kd = 100.0, 5.0
            self.cmd[jn].kp = kp
            self.cmd[jn].kd = kd

        # Clamp torque to actuator limits if provided
        if model.na > 0 and model.actuator_ctrllimited.any():
            u_min = []
            u_max = []
            for aid in self.act_id:
                if model.actuator_ctrllimited[aid]:
                    u_min.append(model.actuator_ctrlrange[aid, 0])
                    u_max.append(model.actuator_ctrlrange[aid, 1])
                else:
                    u_min.append(-np.inf)
                    u_max.append(np.inf)
            self.u_min = np.array(u_min)
            self.u_max = np.array(u_max)
        else:
            self.u_min = np.full(len(self.act_id), -100.0)
            self.u_max = np.full(len(self.act_id),  100.0)

        # Cached model references for foot-space control
        self.base_body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "base_link")
        self.foot_geom_id: Dict[str, int] = {}
        self.leg_prefixes = ["FL", "FR", "RL", "RR"]
        self.side_sign = {"FL": 1.0, "FR": -1.0, "RL": 1.0, "RR": -1.0}
        self.front_sign = {"FL": 1.0, "FR": 1.0, "RL": -1.0, "RR": -1.0}
        base_mat = self.data.xmat[self.base_body_id].reshape(3, 3)
        base_pos = self.data.xpos[self.base_body_id]
        self.foot_home_body: Dict[str, np.ndarray] = {}
        for leg in self.leg_prefixes:
            gid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, leg)
            if gid < 0:
                raise RuntimeError(f"Foot geom '{leg}' not found in model")
            self.foot_geom_id[leg] = gid
            world = self.data.geom_xpos[gid]
            self.foot_home_body[leg] = base_mat.T @ (world - base_pos)

        self.leg_joint_indices: Dict[str, List[int]] = {}
        self.leg_dof_indices: Dict[str, List[int]] = {}
        for leg in self.leg_prefixes:
            idxs = [self.name_to_idx[f"{leg}_hip_joint"],
                    self.name_to_idx[f"{leg}_thigh_joint"],
                    self.name_to_idx[f"{leg}_calf_joint"]]
            self.leg_joint_indices[leg] = idxs
            self.leg_dof_indices[leg] = [self.dof_id[i] for i in idxs]

        self.joint_limits = np.array([self.model.jnt_range[jid] for jid in self.jnt_id])
        self.body_height_ref = self.data.xpos[self.base_body_id][2]
        self.roll_stiff = 0.42
        self.roll_damp = 0.1
        self.pitch_stiff = 0.32
        self.pitch_damp = 0.08
        self.yaw_stiff = 0.18
        self.yaw_damp = 0.06
        self.yaw_place_gain = 0.55
        self.height_stiff = 2.3
        self.sag_vel_gain = 0.16
        self.lat_vel_gain = 0.1
        self.sag_int_gain = 0.28
        self.lat_int_gain = 0.22
        self.integral_limit = 0.1
        self.ik_step_limit = 0.1
        self._dt = self.model.opt.timestep
        self._sag_int = 0.0
        self._lat_int = 0.0
        self._jacp = np.zeros((3, self.model.nv))
        self._jacr = np.zeros((3, self.model.nv))

    def _q(self) -> np.ndarray:
        # current joint positions (in joint order)
        return np.array([self.data.qpos[self.model.jnt_qposadr[jid]] for jid in self.jnt_id])

    def _dq(self) -> np.ndarray:
        # current joint velocities
        return np.array([self.data.qvel[self.model.jnt_dofadr[jid]] for jid in self.jnt_id])

    def _body_rpy(self) -> Tuple[float, float, float]:
        w, x, y, z = self.data.qpos[3:7]
        # MuJoCo quaternions are (w, x, y, z)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (w * y - z * x)
        pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def set_targets(self, targets: Dict[str, float], kp: float = None, kd: float = None):
        for jn, qdes in targets.items():
            jc = self.cmd[jn]
            jc.q = qdes
            jc.dq = 0.0
            jc.tau_ff = 0.0
            if kp is not None:
                jc.kp = kp
            if kd is not None:
                jc.kd = kd
            self.desired_q[self.name_to_idx[jn]] = qdes

    def _compute_torque(self) -> np.ndarray:
        # Refresh bias forces (gravity, Coriolis, etc.) for the current state.
        mj.mj_forward(self.model, self.data)
        bias = np.array([self.data.qfrc_bias[dof] for dof in self.dof_id])
        q = self._q()
        dq = self._dq()
        qdes = np.array([self.cmd[jn].q for jn in self.joint_names])
        dqdes = np.array([self.cmd[jn].dq for jn in self.joint_names])
        kp = np.array([self.cmd[jn].kp for jn in self.joint_names])
        kd = np.array([self.cmd[jn].kd for jn in self.joint_names])
        tau_ff = np.array([self.cmd[jn].tau_ff for jn in self.joint_names])

        tau = kp * (qdes - q) + kd * (dqdes - dq) + tau_ff - bias
        tau = np.clip(tau, self.u_min, self.u_max)
        return tau

    def step(self):
        tau = self._compute_torque()
        self.data.ctrl[self.act_id] = tau
        mj.mj_step(self.model, self.data)

    # ----------------------------
    # Behaviors
    # ----------------------------
    def do_stand(self, settle_time: float = 2.0):
        """Smoothly move to stand pose over 'settle_time' seconds."""
        targets = stand_pose()
        # Start from current q, interpolate to target
        q0 = self._q()
        qT = np.array([targets[jn] for jn in self.joint_names])

        t0 = self.data.time
        while True:
            sim_elapsed = self.data.time - t0
            if settle_time <= 0.0:
                alpha = 1.0
            else:
                alpha = np.clip(sim_elapsed / settle_time, 0.0, 1.0)
            qdes = (1 - alpha) * q0 + alpha * qT
            self.set_targets({jn: qdes[i] for i, jn in enumerate(self.joint_names)})
            self.step()
            if alpha >= 1.0 and (settle_time <= 0.0 or (self.data.time - t0) >= settle_time):
                break
        # Keep holding the stand pose once interpolation finishes.
        self.set_targets(targets)
        self.body_height_ref = self.data.xpos[self.base_body_id][2]
        self._sag_int = 0.0
        self._lat_int = 0.0

    def do_trot(self, duration: float = 6.0, freq: float = 1.5, step_length: float = 0.17,
                step_height: float = 0.085, duty_factor: float = 0.68, lateral_amp: float = 0.009,
                forward_bias: float = 0.015):
        """
        Footspace trot using simple Raibert-style lift and placement with posture compensation.
        """
        phases = {"FR": 0.0, "RL": 0.0, "FL": 0.5, "RR": 0.5}
        start_time = self.data.time
        smoothing = 0.55
        stance_clearance = -0.004

        while (self.data.time - start_time) < duration:
            sim_t = self.data.time
            base_pos = self.data.xpos[self.base_body_id]
            base_rot = self.data.xmat[self.base_body_id].reshape(3, 3)

            roll, pitch, yaw = self._body_rpy()
            wx, wy, wz = self.data.qvel[3:6]
            roll_adj = -(self.roll_stiff * roll + self.roll_damp * wx)
            pitch_adj = -(self.pitch_stiff * pitch + self.pitch_damp * wy)
            roll_adj = float(np.clip(roll_adj, -0.05, 0.05))
            pitch_adj = float(np.clip(pitch_adj, -0.05, 0.05))
            height_err = self.body_height_ref - base_pos[2]
            z_body_offset = float(np.clip(self.height_stiff * height_err, -0.05, 0.05))

            yaw_adj = -(self.yaw_stiff * yaw + self.yaw_damp * wz)
            yaw_adj = float(np.clip(yaw_adj, -0.05, 0.05))

            base_vel_world = self.data.qvel[0:3]
            base_vel_body = base_rot.T @ base_vel_world
            self._sag_int = np.clip(self._sag_int + base_vel_body[0] * self._dt, -self.integral_limit, self.integral_limit)
            self._lat_int = np.clip(self._lat_int + base_vel_body[1] * self._dt, -self.integral_limit, self.integral_limit)
            self._sag_int *= 0.995
            self._lat_int *= 0.995
            sag_vel_comp = -self.sag_vel_gain * base_vel_body[0] - self.sag_int_gain * self._sag_int
            lat_vel_comp = -self.lat_vel_gain * base_vel_body[1] - self.lat_int_gain * self._lat_int
            sag_vel_comp = float(np.clip(sag_vel_comp, -0.09, 0.09))
            lat_vel_comp = float(np.clip(lat_vel_comp, -0.05, 0.05))

            q_current = self._q()
            q_candidate = q_current.copy()

            for leg in self.leg_prefixes:
                home = self.foot_home_body[leg]
                phase = (freq * sim_t + phases[leg]) % 1.0
                if phase < duty_factor:
                    s = phase / duty_factor
                    x = home[0] + step_length * (0.5 - s)
                    z = home[2] + z_body_offset + stance_clearance
                else:
                    s = (phase - duty_factor) / (1.0 - duty_factor)
                    x = home[0] - step_length * (0.5 - s)
                    z = home[2] + z_body_offset + step_height * np.sin(np.pi * s)

                y = home[1]
                y += self.side_sign[leg] * roll_adj
                lateral_wave = lateral_amp * np.sin(2.0 * np.pi * freq * sim_t + phases[leg] * 2.0 * np.pi)
                if phase < duty_factor:
                    y += 0.5 * lateral_wave
                else:
                    y += lateral_wave
                x += self.front_sign[leg] * pitch_adj + forward_bias
                x += sag_vel_comp
                y += lat_vel_comp

                if yaw_adj != 0.0:
                    yaw_place = yaw_adj * self.yaw_place_gain
                    c = np.cos(yaw_place)
                    s = np.sin(yaw_place)
                    x, y = c * x - s * y, s * x + c * y

                target_body = np.array([x, y, z])
                target_world = base_pos + base_rot @ target_body
                foot_id = self.foot_geom_id[leg]
                current_world = self.data.geom_xpos[foot_id]
                delta = target_world - current_world

                self._jacp.fill(0.0)
                self._jacr.fill(0.0)
                mj.mj_jacGeom(self.model, self.data, self._jacp, self._jacr, foot_id)
                dof_idx = self.leg_dof_indices[leg]
                J = self._jacp[:, dof_idx]
                try:
                    dq = np.linalg.solve(J, delta)
                except np.linalg.LinAlgError:
                    dq = np.linalg.lstsq(J, delta, rcond=None)[0]
                dq = np.clip(dq, -self.ik_step_limit, self.ik_step_limit)
                for joint_local, inc in zip(self.leg_joint_indices[leg], dq):
                    q_candidate[joint_local] = q_current[joint_local] + inc

            q_target = smoothing * q_candidate + (1.0 - smoothing) * self.desired_q
            lower = self.joint_limits[:, 0] + 1e-3
            upper = self.joint_limits[:, 1] - 1e-3
            q_target = np.clip(q_target, lower, upper)
            target_map = {jn: q_target[i] for i, jn in enumerate(self.joint_names)}
            self.set_targets(target_map, kp=None, kd=None)
            self.step()


# ---------------------------------------
# Viewer loop (optional)
# ---------------------------------------
def run_with_viewer(ctrl: Go2Controller, mode: str):
    if not HAS_VIEWER:
        print("Viewer not available; run without --viz or install mujoco-python-viewer")
        return

    with viewer.launch_passive(ctrl.model, ctrl.data) as v:
        # v.user_scn.flags[mj.mjtVisFlag.mjVIS_TRANSPARENT] = 1

        # First stand
        ctrl.do_stand(settle_time=2.0)

        # Then optional trot
        if mode == "trot":
            while v.is_running():
                ctrl.do_trot(duration=0.05, freq=1.4)
                v.sync()
        else:
            # Hold stand while viewing
            while v.is_running():
                ctrl.step()
                v.sync()


# ---------------------------------------
# Entrypoint
# ---------------------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["stand", "trot"], default="stand")
    parser.add_argument("--viz", action="store_true", help="Open MuJoCo viewer")
    args = parser.parse_args()

    xml_path = _resolve_go2_xml()
    model = mj.MjModel.from_xml_path(str(xml_path))
    data = mj.MjData(model)
    _reset_to_keyframe(model, data, key_name="home")

    # Build joint name list; validate against model
    jnames = default_joint_order()
    for jn in jnames:
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, jn)
        if jid < 0:
            raise RuntimeError(
                f"Joint '{jn}' not found. Open your XML and adjust default_joint_order() names."
            )

    ctrl = Go2Controller(model, data, jnames)
    ctrl.set_targets(stand_pose())

    if args.viz:
        run_with_viewer(ctrl, args.mode)
        return

    # Headless run (fast stepping)
    if args.mode == "stand":
        ctrl.do_stand(settle_time=2.0)
        # Hold for a bit
        for _ in range(2000):
            ctrl.step()
    elif args.mode == "trot":
        ctrl.do_stand(settle_time=2.0)
        ctrl.do_trot(duration=6.0, freq=1.4)


if __name__ == "__main__":
    main()
