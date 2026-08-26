#!/usr/bin/env python3
"""6-DOF MG995 arm in MuJoCo with control sliders.

Loads arm.urdf, adds position servo actuators (1.13 N*m torque limit,
like a real MG995 @ 6 V), disables self-contact (raw CAD meshes
interpenetrate across joints), then opens the interactive viewer.
Drag the sliders in the Control panel to move the joints.

Run:  .venv-sim/bin/python arm_sim.py
"""
import os
import mujoco
import mujoco.viewer   # explicit submodule import - not exposed by "import mujoco"
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "arm.urdf")
MJCF = os.path.join(HERE, "arm.xml")

# ---------- build model ----------
m = mujoco.MjModel.from_xml_path(URDF)

# save compiled model, then inject servo actuators + solver settings
mujoco.mj_saveLastXML(MJCF, m)
with open(MJCF) as f:
    xml = f.read()

ACTUATORS = """
  <option integrator="implicitfast" timestep="0.002" gravity="0 0 -9.81"/>
  <actuator>
    <position name="J1_base_yaw"   joint="base_first_joint"   kp="8" kv="0.30" forcerange="-1.13 1.13" ctrlrange="-3.14 3.14"/>
    <position name="J2_shoulder"   joint="first_second_joint" kp="8" kv="0.30" forcerange="-1.13 1.13" ctrlrange="-3.14 3.14"/>
    <position name="J3_elbow"      joint="second_third_joint" kp="8" kv="0.30" forcerange="-1.13 1.13" ctrlrange="-3.14 3.14"/>
    <position name="J4_wrist_rot"  joint="third_fourth_joint" kp="6" kv="0.22" forcerange="-1.13 1.13" ctrlrange="-3.14 3.14"/>
    <position name="J5_wrist_tilt" joint="fourth_fifth_joint" kp="6" kv="0.22" forcerange="-1.13 1.13" ctrlrange="-3.14 3.14"/>
    <position name="J6_wrist_roll" joint="fifth_sixth_joint"  kp="6" kv="0.22" forcerange="-1.13 1.13" ctrlrange="-3.14 3.14"/>
  </actuator>
</mujoco>"""
xml = xml.rsplit("</mujoco>", 1)[0] + ACTUATORS
with open(MJCF, "w") as f:
    f.write(xml)

m = mujoco.MjModel.from_xml_path(MJCF)

# raw CAD meshes interpenetrate across joints -> phantom contacts jam
# the servos. Path safety is the user's job; kill all self-contact.
m.geom_contype[:] = 0
m.geom_conaffinity[:] = 0

# planned MG90 metal-gear claw at the tip (~45 g incl. servo), so joint
# droop in the viewer matches the real loaded arm
CLAW_MASS_KG = 0.039
_claw = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "sixth_link")
m.body_mass[_claw] += CLAW_MASS_KG

d = mujoco.MjData(m)

# ---------- headless sanity: servo hold check ----------
d.ctrl[:] = [0.4, -0.3, 0.5, 0.0, 0.3, 0.0]
for _ in range(1500):          # 3 s
    mujoco.mj_step(m, d)
err = float(np.max(np.abs(d.qpos[:6] - d.ctrl[:6])))
print(f"model: nq={m.nq} nu={m.nu} | hold test max err "
      f"{err:.3f} rad ({err*57.3:.1f} deg) {'OK' if err < 0.15 else 'CHECK'}")

# ---------- interactive viewer with sliders ----------
# Native MuJoCo panel gives one slider per actuator (J1..J6).
# On top of that this loop adds live sag/error feedback:
#   - red ghost sphere  = where the claw WOULD be with perfect servos
#   - tip geoms tint green->red with tip error
#   - console line      = per-joint tracking error in degrees
import time

TIP_BODY = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "sixth_link")
tip_geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] == TIP_BODY]
d_tgt = mujoco.MjData(m)          # scratch: kinematics of the COMMANDED pose

print("opening viewer - drag the sliders in the Control panel")
with mujoco.viewer.launch_passive(m, d, show_left_ui=True) as v:
    print("live readout: joint error (deg), tip sag (mm); Ctrl+C here to quit")
    next_print = 0.0
    next_tick = time.time()
    while v.is_running():
        # commanded pose kinematics -> ghost tip position
        d_tgt.qpos[:6] = d.ctrl[:6]
        mujoco.mj_kinematics(m, d_tgt)

        err_deg = np.degrees(d.ctrl[:6] - d.qpos[:6])
        sag_vec = d_tgt.xpos[TIP_BODY] - d.xpos[TIP_BODY]
        sag_mm = float(np.linalg.norm(sag_vec) * 1000)

        # tint tip geoms green -> red over 0..40 mm sag
        t = min(sag_mm / 40.0, 1.0)          # 0 mm -> green, 40 mm -> red
        tip_color = np.array([0.2 + 0.8 * t, 1 - 0.8 * t, 0.2, 0.7])
        for g in tip_geoms:
            m.geom_rgba[g] = tip_color

        # ghost sphere at the ideal tip position
        v.user_scn.ngeom = 1
        mujoco.mjv_initGeom(
            v.user_scn.geoms[0],
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[0.008, 0, 0],
            pos=d_tgt.xpos[TIP_BODY],
            mat=np.eye(3).reshape(-1),
            rgba=[1, 0.15, 0.15, 0.55])

        now = time.time()
        if now >= next_print:
            errs = " ".join(f"J{i+1}:{e:+5.1f}" for i, e in enumerate(err_deg))
            print(f"\r  {errs} | tip sag {sag_mm:4.1f} mm ",
                  end="", flush=True)
            next_print = now + 0.25

        # pace physics to wall-clock real time (dt = 2 ms)
        for _ in range(10):
            mujoco.mj_step(m, d)
        next_tick += 0.02
        time.sleep(max(0.0, next_tick - time.time()))
        if next_tick - time.time() < -1.0:
            next_tick = time.time()      # fell behind, don't spiral
        v.sync()
print("\nclosed.")
