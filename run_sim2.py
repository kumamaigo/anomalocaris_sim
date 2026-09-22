import math
import time
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("rod_model.xml")
data = mujoco.MjData(model)

NUM_RODS = 6
FREQUENCY = 1.0  # 1秒間に1周期
AMPLITUDE = 30.0  # 上下の振り幅（度）
PHASE_SHIFT = 0.6  # 前後の棒の位相差（うねり波の波長）
SIM_SPEED = 0.25

print("🚀 棒モデルの波動シミュレーションを開始します。")

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.distance = 0.8
    viewer.cam.elevation = -30

    while viewer.is_running():
        step_start = time.time()
        sim_time = data.time

        for i in range(1, NUM_RODS + 1):
            phase = (i - 1) * PHASE_SHIFT
            target_deg = AMPLITUDE * math.sin(
                2 * math.pi * FREQUENCY * sim_time - phase
            )

            act_id_L = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"motor_L_{i}"
            )
            act_id_R = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"motor_R_{i}"
            )

            if act_id_L != -1:
                data.ctrl[act_id_L] = target_deg
            if act_id_R != -1:
                data.ctrl[act_id_R] = target_deg  # 左右同位相で波打ち

        mujoco.mj_step(model, data)
        viewer.sync()

        elapsed = time.time() - step_start
        target_delay = model.opt.timestep / SIM_SPEED
        if target_delay > elapsed:
            time.sleep(target_delay - elapsed)