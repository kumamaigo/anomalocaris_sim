import math
import mujoco
import mujoco.viewer

# ==========================================
# 📍 位置調整用パラメータ（胴体固定状態で微調整）
# ==========================================
X_START = 0.110      # 先頭ヒレの位置（数値を増減させて前後を合わせます）
X_STEP = 0.055       # ヒレの間隔（くぼみのピッチ）
Y_LEFT = 0.030       # 左ヒレの幅方向の位置（奥・手前）
Y_RIGHT = -0.030     # 右ヒレの幅方向の位置
# ==========================================

def generate_xml():
    left_fins_xml = ""
    right_fins_xml = ""
    actuators_xml = ""

    for i in range(6):
        x_pos_l = X_START - (i * X_STEP)
        x_pos_r = x_pos_l
        
        # 左ヒレ
        left_fins_xml += f"""
      <body name="fin_l{i}" pos="{x_pos_l:.4f} {Y_LEFT:.4f} 0">
        <joint name="joint_l{i}" type="hinge" axis="1 0 0" range="-45 45"/>
        <geom type="mesh" mesh="fin_mesh" rgba="0.8 0.3 0.2 1" contype="0" conaffinity="0"/>
      </body>"""

        # 右ヒレ
        right_fins_xml += f"""
      <body name="fin_r{i}" pos="{x_pos_r:.4f} {Y_RIGHT:.4f} 0">
        <joint name="joint_r{i}" type="hinge" axis="1 0 0" range="-45 45"/>
        <geom type="mesh" mesh="fin_mesh" euler="0 0 180" rgba="0.2 0.8 0.3 1" contype="0" conaffinity="0"/>
      </body>"""

        actuators_xml += f"""
    <position joint="joint_l{i}" kp="50"/>
    <position joint="joint_r{i}" kp="30"/>"""

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
    <body name="torso" pos="0 0 1">
      <geom type="mesh" mesh="torso_mesh" rgba="0.2 0.6 0.8 1" contype="0" conaffinity="0"/>
      {left_fins_xml}
      {right_fins_xml}
    </body>
  </worldbody>

  <actuator>{actuators_xml}
  </actuator>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(generate_xml())
data = mujoco.MjData(model)

FREQ = 1.5
AMPLITUDE = 0.5
PHASE_SHIFT = 0.6

def cpg_controller(m, d):
    t = d.time
    for i in range(6):
        phase = 2 * math.pi * FREQ * t - (i * PHASE_SHIFT)
        angle = math.sin(phase) * AMPLITUDE
        d.ctrl[i * 2] = angle
        d.ctrl[i * 2 + 1] = -angle

mujoco.set_mjcb_control(cpg_controller)
mujoco.viewer.launch(model, data)