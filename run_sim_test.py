from rough_terrain_env import RoughTerrainEnv
import numpy as np
import time

env = RoughTerrainEnv(gui=True, log_data=True)

# ---------------------
#  TEST 1: RANDOM POLICY
# ---------------------
print("\n=== TEST 1: RANDOM ROLLOUT ===")
obs = env.reset(random_start=True)

print("Observation keys:", obs.keys())
print("Height patch shape:", obs["height_patch"].shape)
print("Gravity (body frame):", obs["gravity_body"])


done = False

while not done:
    action = np.array([
        np.random.uniform(-env.max_throttle, env.max_throttle),
        np.random.uniform(-env.max_steering, env.max_steering)
    ])
    obs, reward, done, info = env.step(action)
    time.sleep(1/240)

print("Episode ended with termination:", info)
dataset = env.get_episode_dataset()
print("Logged steps:", dataset["actions"].shape[0])

dataset = env.get_episode_dataset()
print(dataset["actions"].shape)
print(dataset["height_patches"].shape)

# -----------------------------
# TEST 2: TRACKING CONTROLLER
# -----------------------------
print("\n=== TEST 2: TRACKING CONTROLLER ===")
obs = env.reset(random_start=True)
done = False

while not done:
    # Reference = drive towards the goal
    state = env.get_state()
    pos = state["position"]
    ref = [env.goal_xy[0], env.goal_xy[1], 0]  # x_goal, y_goal, yaw=0

    action = env.tracking_controller(ref, state)
    obs, reward, done, info = env.step(action)
    time.sleep(1/240)

print("Tracking episode ended:", info)

env.disconnect()
