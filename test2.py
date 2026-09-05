import math
import time
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

# 一時停止状態を管理するフラグ
paused = False

# キーが押された時に呼ばれる処理（Spaceキーでpausedを反転）
def key_callback(keycode):
    global paused
    if keycode == 32:  # 32 は Space キーのコード
        paused = not paused

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    while viewer.is_running():
        step_start = time.time()

        # 一時停止中でない場合のみ計算を進める
        if not paused:
            # data.time（シミュレーション内部時間）を使うことで停止・再開がスムーズになります
            target_angle = math.sin(data.time * 5) * 0.5

            data.ctrl[0] = target_angle   # 左ヒレ
            data.ctrl[1] = -target_angle  # 右ヒレ

            mujoco.mj_step(model, data)

        viewer.sync()
        time.sleep(max(0, 0.002 - (time.time() - step_start)))