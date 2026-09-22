import math
import time
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("fin_model.xml")
data = mujoco.MjData(model)

num_actuators = model.nu

FREQUENCY = 0.8
AMPLITUDE = 35.0
PHASE_SHIFT = 0.5
SIM_SPEED = 0.25

print(f"🚀 全 {num_actuators} 枚のヒレでシミュレーションを開始します。")

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.distance = 0.5

    while viewer.is_running():
        step_start = time.time()
        sim_time = data.time

        for i in range(1, num_actuators + 1):
            phase = (i - 1) * PHASE_SHIFT
            target_deg = AMPLITUDE * math.sin(
                2 * math.pi * FREQUENCY * sim_time - phase
            )

            act_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"motor_{i}"
            )
            joint_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_a{i}"
            )

            if act_id != -1 and joint_id != -1:
                curr_deg = data.qpos[joint_id] * (180.0 / math.pi)
                data.ctrl[act_id] = (target_deg - curr_deg) * 0.15

        mujoco.mj_step(model, data)
        viewer.sync()

        elapsed = time.time() - step_start
        target_delay = model.opt.timestep / SIM_SPEED
        if target_delay > elapsed:
            time.sleep(target_delay - elapsed)