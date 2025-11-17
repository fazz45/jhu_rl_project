from rough_terrain_env import RoughTerrainEnv
import numpy as np
import time
import os

# ---------------------------
# User settings
# ---------------------------
num_episodes = 3
steps_per_episode = 20000
output_file = "small_dataset.npz"

# ---------------------------
# Initialize the environment
# ---------------------------
env = RoughTerrainEnv(gui=True, log_data=True, max_steps=steps_per_episode)

all_gravity = []
all_vbody = []
all_wbody = []
all_height = []
all_actions = []
all_rewards = []
all_dones = []

print(f"\n=== COLLECTING {num_episodes} EPISODES ===\n")

for ep in range(num_episodes):
    print(f"\n--- Episode {ep+1}/{num_episodes} ---")
    obs = env.reset(random_start=True)
    done = False

    while not done:
        # random actions for data collection
        action = np.array([
            np.random.uniform(-env.max_throttle, env.max_throttle),
            np.random.uniform(-env.max_steering, env.max_steering)
        ], dtype=np.float32)

        obs, reward, done, info = env.step(action)

        time.sleep(1/240)  # slow down for visualization

    # ---- retrieve episode dataset ----
    ep_data = env.get_episode_dataset()

    all_gravity.append(ep_data["gravity_body"])
    all_vbody.append(ep_data["v_body"])
    all_wbody.append(ep_data["w_body"])
    all_height.append(ep_data["height_patches"])
    all_actions.append(ep_data["actions"])
    all_rewards.append(ep_data["rewards"])
    all_dones.append(ep_data["dones"])

    print(f" Episode length: {ep_data['actions'].shape[0]} steps")

# ---------------------------
# Save dataset
# ---------------------------
print("\n=== SAVING DATASET ===")

np.savez(
    output_file,
    gravity=np.array(all_gravity, dtype=object),
    v_body=np.array(all_vbody, dtype=object),
    w_body=np.array(all_wbody, dtype=object),
    height=np.array(all_height, dtype=object),
    actions=np.array(all_actions, dtype=object),
    rewards=np.array(all_rewards, dtype=object),
    dones=np.array(all_dones, dtype=object),
)

print(f"\nSaved dataset to {output_file}\n")

# ---------------------------
# Show summary
# ---------------------------
print("=== DATASET SUMMARY ===")
print("Gravity shape:", np.array(all_gravity, dtype=object).shape)
print("V-body shape:", np.array(all_vbody, dtype=object).shape)
print("W-body shape:", np.array(all_wbody, dtype=object).shape)
print("Height patches shape:", np.array(all_height, dtype=object).shape)
print("Actions shape:", np.array(all_actions, dtype=object).shape)
print("Rewards shape:", np.array(all_rewards, dtype=object).shape)

env.disconnect()
print("\n=== DONE ===")
