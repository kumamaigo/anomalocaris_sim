import math
import mujoco
import mujoco.viewer

# 片側4枚（計8枚）のヒレを持つアノマロカリス型ダミーモデル
xml = """
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <light pos="0 0 3"/>

    <!-- 長い胴体 -->
    <body name="torso" pos="0 0 1">
      <freejoint/>
      <geom type="box" size="0.4 0.08 0.04" rgba="0.2 0.6 0.8 1"/>

      <!-- ヒレ 0 (最前部) -->
      <body name="fin_l0" pos="0.3 0.08 0">
        <joint name="joint_l0" type="hinge" axis="1 0 0" range="-0.8 0.8"/>
        <geom type="box" size="0.06 0.12 0.008" pos="0 0.12 0" rgba="0.9 0.2 0.2 1"/>
      </body>
      <body name="fin_r0" pos="0.3 -0.08 0">
        <joint name="joint_r0" type="hinge" axis="1 0 0" range="-0.8 0.8"/>
        <geom type="box" size="0.06 0.12 0.008" pos="0 -0.12 0" rgba="0.2 0.9 0.2 1"/>
      </body>

      <!-- ヒレ 1 -->
      <body name="fin_l1" pos="0.1 0.08 0">
        <joint name="joint_l1" type="hinge" axis="1 0 0" range="-0.8 0.8"/>
        <geom type="box" size="0.06 0.12 0.008" pos="0 0.12 0" rgba="0.8 0.3 0.2 1"/>
      </body>
      <body name="fin_r1" pos="0.1 -0.08 0">
        <joint name="joint_r1" type="hinge" axis="1 0 0" range="-0.8 0.8"/>
        <geom type="box" size="0.06 0.12 0.008" pos="0 -0.12 0" rgba="0.2 0.8 0.3 1"/>
      </body>

      <!-- ヒレ 2 -->
      <body name="fin_l2" pos="-0.1 0.08 0">
        <joint name="joint_l2" type="hinge" axis="1 0 0" range="-0.8 0.8"/>
        <geom type="box" size="0.06 0.12 0.008" pos="0 0.12 0" rgba="0.7 0.4 0.2 1"/>
      </body>
      <body name="fin_r2" pos="-0.1 -0.08 0">
        <joint name="joint_r2" type="hinge" axis="1 0 0" range="-0.8 0.8"/>
        <geom type="box" size="0.06 0.12 0.008" pos="0 -0.12 0" rgba="0.2 0.7 0.4 1"/>
      </body>

      <!-- ヒレ 3 (最後部) -->
      <body name="fin_l3" pos="-0.3 0.08 0">
        <joint name="joint_l3" type="hinge" axis="1 0 0" range="-0.8 0.8"/>
        <geom type="box" size="0.06 0.12 0.008" pos="0 0.12 0" rgba="0.6 0.5 0.2 1"/>
      </body>
      <body name="fin_r3" pos="-0.3 -0.08 0">
        <joint name="joint_r3" type="hinge" axis="1 0 0" range="-0.8 0.8"/>
        <geom type="box" size="0.06 0.12 0.008" pos="0 -0.12 0" rgba="0.2 0.6 0.5 1"/>
      </body>
    </body>
  </worldbody>

  <!-- 8個の関節モータ -->
  <actuator>
    <position joint="joint_l0" kp="10"/>
    <position joint="joint_r0" kp="10"/>
    <position joint="joint_l1" kp="10"/>
    <position joint="joint_r1" kp="10"/>
    <position joint="joint_l2" kp="10"/>
    <position joint="joint_r2" kp="10"/>
    <position joint="joint_l3" kp="10"/>
    <position joint="joint_r3" kp="10"/>
  </actuator>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# CPG (波動生成) の重要パラメータ
FREQ = 1.5         # 周波数 (Hz): 1秒間にヒレが往復する回数
AMPLITUDE = 0.5    # 振幅 (rad): ヒレの最大仰ぎ角
PHASE_SHIFT = 0.8  # 位相差 (rad): 前後のヒレ同士の動くタイミングのズレ

def cpg_controller(m, d):
    t = d.time
    for i in range(4):
        # i（0:最前線 〜 3:最後尾）に応じて位相(Phase)を少しずつ遅らせる
        phase = 2 * math.pi * FREQ * t - (i * PHASE_SHIFT)
        angle = math.sin(phase) * AMPLITUDE

        # 左ヒレ (インデックス: 0, 2, 4, 6)
        d.ctrl[i * 2] = angle
        # 右ヒレ (インデックス: 1, 3, 5, 7) - 逆位相で力強く羽ばたく
        d.ctrl[i * 2 + 1] = -angle

mujoco.set_mjcb_control(cpg_controller)
mujoco.viewer.launch(model, data)