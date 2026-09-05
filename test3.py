# this code can use GUI
import math
import mujoco
import mujoco.viewer

xml = """
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <light pos="0 0 3"/>
    <body name="torso" pos="0 0 1">
      <freejoint/>
      <geom type="box" size="0.3 0.1 0.05" rgba="0.2 0.6 0.8 1"/>
      <body name="fin_left" pos="0 0.1 0">
        <joint name="joint_left" type="hinge" axis="1 0 0" range="-0.8 0.8"/>
        <geom type="box" size="0.1 0.15 0.01" pos="0 0.15 0" rgba="0.8 0.2 0.2 1"/>
      </body>
      <body name="fin_right" pos="0 -0.1 0">
        <joint name="joint_right" type="hinge" axis="1 0 0" range="-0.8 0.8"/>
        <geom type="box" size="0.1 0.15 0.01" pos="0 -0.15 0" rgba="0.2 0.8 0.2 1"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position joint="joint_left" kp="10"/>
    <position joint="joint_right" kp="10"/>
  </actuator>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# 計算のたびに呼ばれる制御関数を登録
def custom_controller(m, d):
    target_angle = math.sin(d.time * 5) * 0.5
    d.ctrl[0] = target_angle
    d.ctrl[1] = -target_angle

mujoco.set_mjcb_control(custom_controller)

# GUI主導のビューアを起動（PauseボタンやResetボタン、Spaceキーがそのまま動作します）
mujoco.viewer.launch(model, data)