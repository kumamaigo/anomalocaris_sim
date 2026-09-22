import math
import sys
import time
import mujoco
import mujoco.viewer

# ==========================================
# 📍 動作調整用パラメータ
# ==========================================
AMPLITUDE_DEG = 20.0  # 💡 振り幅（片側20度）
FREQ = 1.0 / 3.0  # 1波に3秒かける
SIM_SPEED = 1.0  # 等倍速
PHASE_SHIFT = 0.5  # 位相差

X_START = 0.110
X_STEP = 0.055
Y_LEFT = 0.030
Y_RIGHT = -0.030
Z_BODY = 1.0
FIN_LENGTH = 0.12
# ==========================================


def generate_xml():
    left_fins_xml = ""
    right_fins_xml = ""
    actuators_xml = ""

    skin_vertices_l = ""
    skin_vertices_r = ""
    skin_bones_l = ""
    skin_bones_r = ""
    skin_faces_l = ""
    skin_faces_r = ""

    for i in range(6):
        x_pos = X_START - (i * X_STEP)

        # 💡 armature="0.005" を追加して慣性を付与し、数値的発振（暴れ）を物理的に抑制
        left_fins_xml += f"""
      <body name="fin_l{i}" pos="{x_pos:.4f} {Y_LEFT:.4f} 0">
        <joint name="joint_l{i}" type="hinge" axis="1 0 0" range="-60 60" armature="0.005" damping="0.1"/>
        <geom type="mesh" mesh="fin_mesh" rgba="0.8 0.3 0.2 1" contype="0" conaffinity="0"/>
      </body>"""

        right_fins_xml += f"""
      <body name="fin_r{i}" pos="{x_pos:.4f} {Y_RIGHT:.4f} 0">
        <joint name="joint_r{i}" type="hinge" axis="1 0 0" range="-60 60" armature="0.005" damping="0.1"/>
        <geom type="mesh" mesh="fin_mesh" euler="0 0 180" rgba="0.2 0.8 0.3 1" contype="0" conaffinity="0"/>
      </body>"""

        # 💡 適切な制御ゲイン (kp=5) に調整
        actuators_xml += f"""
    <position joint="joint_l{i}" kp="5"/>
    <position joint="joint_r{i}" kp="5"/>"""

        v_base_l = f" {x_pos:.4f} {Y_LEFT:.4f} {Z_BODY:.4f}"
        v_tip_l = f" {x_pos:.4f} {Y_LEFT + FIN_LENGTH:.4f} {Z_BODY:.4f}"
        skin_vertices_l += v_base_l + v_tip_l
        skin_bones_l += f"""
      <bone body="fin_l{i}" bindpos="{x_pos:.4f} {Y_LEFT:.4f} {Z_BODY:.4f}" bindquat="1 0 0 0" vertid="{i*2} {i*2+1}" vertweight="1.0 1.0"/>"""

        v_base_r = f" {x_pos:.4f} {Y_RIGHT:.4f} {Z_BODY:.4f}"
        v_tip_r = f" {x_pos:.4f} {Y_RIGHT - FIN_LENGTH:.4f} {Z_BODY:.4f}"
        skin_vertices_r += v_base_r + v_tip_r
        skin_bones_r += f"""
      <bone body="fin_r{i}" bindpos="{x_pos:.4f} {Y_RIGHT:.4f} {Z_BODY:.4f}" bindquat="1 0 0 0" vertid="{i*2} {i*2+1}" vertweight="1.0 1.0"/>"""

        if i < 5:
            b1, t1 = i * 2, i * 2 + 1
            b2, t2 = (i + 1) * 2, (i + 1) * 2 + 1
            skin_faces_l += f" {b1} {t1} {t2}  {b1} {t2} {b2}"
            skin_faces_r += f" {b1} {t2} {t1}  {b1} {b2} {t2}"

    return f"""
<mujoco>
  <compiler angle="degree"/>
  <option gravity="0 0 0"/>

  <asset>
    <mesh name="torso_mesh" file="assets/torso.stl" scale="0.001 0.001 0.001"/>
    <mesh name="fin_mesh" file="assets/fin.stl" scale="0.001 0.001 0.001"/>
  </asset>

  <worldbody>
    <light pos="0 0 3"/>
    <body name="torso" pos="0 0 {Z_BODY}">
      <geom type="mesh" mesh="torso_mesh" rgba="0.2 0.6 0.8 1" contype="0" conaffinity="0"/>
      {left_fins_xml}
      {right_fins_xml}
    </body>
  </worldbody>

  <deformable>
    <skin name="membrane_left" rgba="0.1 0.8 0.9 0.6" vertex="{skin_vertices_l}" face="{skin_faces_l}">
      {skin_bones_l}
    </skin>
    <skin name="membrane_right" rgba="0.1 0.8 0.9 0.6" vertex="{skin_vertices_r}" face="{skin_faces_r}">
      {skin_bones_r}
    </skin>
  </deformable>

  <actuator>
    {actuators_xml}
  </actuator>
</mujoco>
"""


model = mujoco.MjModel.from_xml_string(generate_xml())
data = mujoco.MjData(model)

print("🚀 安定化修正版を起動します...")

try:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat = [0.0, 0.0, Z_BODY]
        viewer.cam.distance = 0.65
        viewer.cam.elevation = -30
        viewer.cam.azimuth = 135

        while viewer.is_running():
            step_start = time.time()

            t = data.time
            for i in range(6):
                phase = 2 * math.pi * FREQ * t - (i * PHASE_SHIFT)

                # 💡 Python API (data.ctrl) にはラジアンで値を渡します
                angle_deg = math.sin(phase) * AMPLITUDE_DEG
                angle_rad = math.radians(angle_deg)

                data.ctrl[i * 2] = angle_rad
                data.ctrl[i * 2 + 1] = -angle_rad

            mujoco.mj_step(model, data)
            viewer.sync()

            elapsed = time.time() - step_start
            target_delay = model.opt.timestep / SIM_SPEED
            if target_delay > elapsed:
                remaining = target_delay - elapsed
                while remaining > 0 and viewer.is_running():
                    sleep_time = min(remaining, 0.01)
                    time.sleep(sleep_time)
                    remaining -= sleep_time

except KeyboardInterrupt:
    pass

print("🛑 シミュレーションを正常に終了しました。")
sys.exit(0)