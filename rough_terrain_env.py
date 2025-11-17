# import pybullet as p
# import pybullet_data
# import time
# import numpy as np
# import os


# class RoughTerrainEnv:
#     def __init__(
#         self,
#         terrain_npy_path="jezero_128x128_1m.npy",
#         texture_path="jezero_texture_128x128.png",
#         gui=True,
#         max_steps=500,
#         log_data=False,
#         height_window_size=16,
#     ):
#         """
#         Rough terrain rover environment with:
#           - reset / step
#           - state / observation
#           - logging
#           - simple tracking controller
#         """

#         self.gui = gui
#         self.max_steps = max_steps
#         self.log_data = log_data
#         self.height_window_size = height_window_size

#         # connect to PyBullet
#         self.client_id = p.connect(p.GUI if gui else p.DIRECT)
#         p.setAdditionalSearchPath(pybullet_data.getDataPath())
#         p.setGravity(0, 0, -3.71)

#         # terrain parameters
#         self.terrain_size = 128
#         self.pixel_size_m = 1.0096633268711
#         self.terrain_scale_xy = self.pixel_size_m

#         # load heightmap
#         assert os.path.exists(terrain_npy_path), f"Missing {terrain_npy_path}"
#         heights = np.load(terrain_npy_path).astype(np.float32)
#         assert heights.shape == (self.terrain_size, self.terrain_size)
#         self.heights = heights

#         # vertical exaggeration if needed
#         hmin, hmax = float(np.nanmin(self.heights)), float(np.nanmax(self.heights))
#         hrange = hmax - hmin
#         print(f"[heightmap] min={hmin:.3f} max={hmax:.3f} range={hrange:.3f}")
#         if hrange < 0.5:
#             print("[heightmap] small range; applying vertical exaggeration ×50")
#             self.heights *= 50.0

#         # recompute stats and z offset
#         hmin_f, hmax_f = float(np.nanmin(self.heights)), float(np.nanmax(self.heights))
#         self.z_offset = (hmin_f + hmax_f) / 2.0
#         print(f"[heightmap] Final min={hmin_f:.3f} max={hmax_f:.3f}")
#         print(f"[heightmap] Applying Z-offset to terrain body: {self.z_offset:.3f}")

#         # flatten for pybullet
#         height_data = self.heights.reshape(-1).tolist()

#         # create terrain collision
#         print("Creating collision shape...")
#         terrain_shape = p.createCollisionShape(
#             shapeType=p.GEOM_HEIGHTFIELD,
#             meshScale=[self.terrain_scale_xy, self.terrain_scale_xy, 1.0],
#             heightfieldData=height_data,
#             heightfieldTextureScaling=self.terrain_size - 1,
#             numHeightfieldRows=self.terrain_size,
#             numHeightfieldColumns=self.terrain_size,
#         )

#         print("Creating terrain body...")
#         self.terrain_body = p.createMultiBody(
#             baseMass=0, baseCollisionShapeIndex=terrain_shape
#         )

#         p.resetBasePositionAndOrientation(
#             self.terrain_body, [0, 0, self.z_offset], [0, 0, 0, 1]
#         )
#         p.changeDynamics(self.terrain_body, -1, lateralFriction=1.2)

#         # texture
#         if os.path.exists(texture_path):
#             print("Loading Mars texture...")
#             texture_id = p.loadTexture(texture_path)
#             p.changeVisualShape(self.terrain_body, -1, textureUniqueId=texture_id)
#         else:
#             print(f"[warn] texture {texture_path} not found, using default color")

#         # robot config
#         self.robot_id = None
#         self.rear_wheels = [2, 3]
#         self.front_wheels = [5, 7]
#         self.steering_joints = [4, 6]
#         self.all_wheels = self.rear_wheels + self.front_wheels

#         # action limits
#         self.max_throttle = 40.0 #ear;ier 15
#         self.max_steering = 0.6

#         # goal (x, y) in world frame
#         self.goal_xy = np.array([20.0, 0.0])

#         # logging buffers
#         self.episode_log = []
#         self.step_count = 0

#     # ------------------------------------------------------------------
#     # helpers for height lookup and local patch
#     # ------------------------------------------------------------------
#     def world_to_grid(self, x, y):
#         """Convert world (x,y) to heightmap indices (row, col)."""
#         i = int(np.round(x / self.terrain_scale_xy + self.terrain_size / 2))
#         j = int(np.round(y / self.terrain_scale_xy + self.terrain_size / 2))
#         i = np.clip(i, 0, self.terrain_size - 1)
#         j = np.clip(j, 0, self.terrain_size - 1)
#         return j, i  # row, col

#     def get_height_at(self, x, y):
#         row, col = self.world_to_grid(x, y)
#         return float(self.heights[row, col])

#     def get_local_height_patch(self, x, y):
#         """
#         Returns a small height patch around (x,y) in robot-centric world coords.
#         Shape: (window_size, window_size)
#         """
#         ws = self.height_window_size
#         row_c, col_c = self.world_to_grid(x, y)
#         half = ws // 2

#         row_min = max(0, row_c - half)
#         row_max = min(self.terrain_size, row_c + half)
#         col_min = max(0, col_c - half)
#         col_max = min(self.terrain_size, col_c + half)

#         patch = np.zeros((ws, ws), dtype=np.float32)
#         sub = self.heights[row_min:row_max, col_min:col_max]
#         patch[
#             (row_min - (row_c - half)) : (row_max - (row_c - half)),
#             (col_min - (col_c - half)) : (col_max - (col_c - half)),
#         ] = sub
#         return patch

#     # ------------------------------------------------------------------
#     # core env API
#     # ------------------------------------------------------------------
#     def reset(self, random_start=True):
#         """Reset robot pose and episode log. Returns initial observation."""
#         if self.robot_id is not None:
#             p.removeBody(self.robot_id)

#         # choose start position
#         if random_start:
#             # small random patch near center
#             start_x = np.random.uniform(-5.0, 5.0)
#             start_y = np.random.uniform(-5.0, 5.0)
#         else:
#             start_x, start_y = 0.0, 0.0

#         hz = self.get_height_at(start_x, start_y)
#         start_z = hz + 0.8

#         start_pos = [start_x, start_y, start_z]
#         start_orientation = p.getQuaternionFromEuler([0, 0, 0])

#         print("Loading robot...")
#         self.robot_id = p.loadURDF(
#             "racecar/racecar.urdf", start_pos, start_orientation
#         )

#         p.changeDynamics(self.robot_id, -1, lateralFriction=1.0)

#         # keep camera roughly on the robot / center
#         p.resetDebugVisualizerCamera(
#             cameraDistance=12.0,
#             cameraYaw=45.0,
#             cameraPitch=-25.0,
#             cameraTargetPosition=[start_x, start_y, hz],
#         )

#         self.step_count = 0
#         self.episode_log = []

#         obs = self.get_observation()
#         return obs

#     def step(self, action):
#         """
#         Take one step in the environment.

#         action: np.array([throttle, steering])

#         Returns: obs, reward, done, info
#         """

#         throttle = float(np.clip(action[0], -self.max_throttle, self.max_throttle))
#         steering = float(np.clip(action[1], -self.max_steering, self.max_steering))

#         self._apply_action(throttle, steering)

#         p.stepSimulation()
#         self.step_count += 1

#         obs_next = self.get_observation()
#         state_next = self.get_state()

#         # reward = negative distance to goal
#         pos = state_next["position"]
#         xy = np.array(pos[:2])
#         dist_to_goal = np.linalg.norm(xy - self.goal_xy)
#         reward = -dist_to_goal

#         # termination conditions
#         done = False
#         info = {"dist_to_goal": dist_to_goal}

#         # robot height vs terrain
#         base_z = pos[2]
#         ground_z = self.get_height_at(pos[0], pos[1])
#         if base_z - ground_z > 3.0:  # big jump, probably flipped or flying
#             done = True
#             info["termination"] = "unstable_height"
#         if dist_to_goal < 1.0:
#             done = True
#             info["termination"] = "goal_reached"
#         if self.step_count >= self.max_steps:
#             done = True
#             info["termination"] = "max_steps"

#         # simple logging
#         if self.log_data:
#             self.episode_log.append(
#                 {
#                     "obs": obs_next,
#                     "action": np.array([throttle, steering], dtype=np.float32),
#                     "reward": reward,
#                     "done": done,
#                 }
#             )

#         return obs_next, reward, done, info

#     # ------------------------------------------------------------------
#     # state and observation
#     # ------------------------------------------------------------------
#     def get_state(self):
#         """Returns a dict with pose and body twist in world frame."""
#         assert self.robot_id is not None, "Robot not loaded yet"
#         pos, orn = p.getBasePositionAndOrientation(self.robot_id)
#         lin_vel, ang_vel = p.getBaseVelocity(self.robot_id)

#         state = {
#             "position": np.array(pos, dtype=np.float32),
#             "orientation_quat": np.array(orn, dtype=np.float32),
#             "linear_velocity": np.array(lin_vel, dtype=np.float32),
#             "angular_velocity": np.array(ang_vel, dtype=np.float32),
#         }
#         return state

#     def get_observation(self):
#         """
#         Observation used by the dynamics model:
#           - gravity vector in body frame (tilt)
#           - body twist in body frame (v in R^6)
#           - local height map around the robot
#         """
#         st = self.get_state()
#         pos = st["position"]
#         quat = st["orientation_quat"]
#         lin_vel_world = st["linear_velocity"]
#         ang_vel_world = st["angular_velocity"]

#         # rotation matrix from quaternion
#         rot = np.array(p.getMatrixFromQuaternion(quat), dtype=np.float32).reshape(3, 3)

#         g_world = np.array([0.0, 0.0, -1.0], dtype=np.float32)
#         g_body = rot.T @ g_world  # gravity vector in body frame

#         v_body = rot.T @ lin_vel_world
#         w_body = rot.T @ ang_vel_world

#         height_patch = self.get_local_height_patch(pos[0], pos[1])

#         obs = {
#             "gravity_body": g_body,
#             "v_body": v_body,
#             "w_body": w_body,
#             "height_patch": height_patch,
#             "position_xy": pos[:2],
#         }
#         return obs

#     # ------------------------------------------------------------------
#     # actuation and simple tracking controller
#     # ------------------------------------------------------------------
#     def _apply_action(self, throttle, steering):
#         """Low-level mapping from throttle/steering to wheel and steering joints."""
#         # steering
#         for j in self.steering_joints:
#             p.setJointMotorControl2(
#                 bodyUniqueId=self.robot_id,
#                 jointIndex=j,
#                 controlMode=p.POSITION_CONTROL,
#                 targetPosition=steering,
#                 force=5.0,
#             )

#         # wheels
#         for j in self.all_wheels:
#             p.setJointMotorControl2(
#                 bodyUniqueId=self.robot_id,
#                 jointIndex=j,
#                 controlMode=p.VELOCITY_CONTROL,
#                 targetVelocity=throttle,
#                 force=50.0, #earlier 20
#             )

#     def tracking_controller(self, ref_pose, state=None):
#         """
#         Very simple tracking controller.

#         ref_pose: [x_ref, y_ref, yaw_ref]
#         state: if None, uses current state.

#         Returns: np.array([throttle, steering])
#         """
#         if state is None:
#             state = self.get_state()

#         pos = state["position"]
#         quat = state["orientation_quat"]
#         yaw = p.getEulerFromQuaternion(quat)[2]

#         x, y = pos[0], pos[1]
#         x_ref, y_ref, yaw_ref = ref_pose

#         # errors
#         dx = x_ref - x
#         dy = y_ref - y
#         dist = np.hypot(dx, dy)
#         desired_heading = np.arctan2(dy, dx)
#         heading_error = desired_heading - yaw

#         # wrap to [-pi, pi]
#         heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

#         # simple PD-like law
#         k_v = 1.0
#         k_yaw = 1.5

#         throttle = np.clip(k_v * dist, -self.max_throttle, self.max_throttle)
#         steering = np.clip(k_yaw * heading_error, -self.max_steering, self.max_steering)

#         return np.array([throttle, steering], dtype=np.float32)

#     # ------------------------------------------------------------------
#     # dataset helpers
#     # ------------------------------------------------------------------
#     def get_episode_dataset(self):
#         """
#         Returns numpy arrays (obs, actions, rewards, dones) for the last episode.
#         You can extend this to save to disk.
#         """
#         if not self.episode_log:
#             return None

#         actions = np.stack([step["action"] for step in self.episode_log], axis=0)
#         rewards = np.array([step["reward"] for step in self.episode_log], dtype=np.float32)
#         dones = np.array([step["done"] for step in self.episode_log], dtype=np.bool_)

#         # for obs we just keep gravity/v_body/w_body and flatten the height patch
#         gravity = np.stack(
#             [step["obs"]["gravity_body"] for step in self.episode_log], axis=0
#         )
#         v_body = np.stack(
#             [step["obs"]["v_body"] for step in self.episode_log], axis=0
#         )
#         w_body = np.stack(
#             [step["obs"]["w_body"] for step in self.episode_log], axis=0
#         )
#         height_patches = np.stack(
#             [step["obs"]["height_patch"] for step in self.episode_log], axis=0
#         )

#         dataset = {
#             "gravity_body": gravity,
#             "v_body": v_body,
#             "w_body": w_body,
#             "height_patches": height_patches,
#             "actions": actions,
#             "rewards": rewards,
#             "dones": dones,
#         }
#         return dataset

#     def disconnect(self):
#         p.disconnect(self.client_id)


# # ----------------------------------------------------------------------
# # example usage: random rollout and one tracking rollout
# # ----------------------------------------------------------------------
# if __name__ == "__main__":
#     env = RoughTerrainEnv(gui=True, log_data=True)

#     print("--- Random policy rollout ---")
#     obs = env.reset(random_start=True)
#     done = False
#     total_reward = 0.0

#     while not done:
#         # random actions
#         action = np.array(
#             [
#                 np.random.uniform(-env.max_throttle, env.max_throttle),
#                 np.random.uniform(-env.max_steering, env.max_steering),
#             ],
#             dtype=np.float32,
#         )
#         obs, reward, done, info = env.step(action)
#         total_reward += reward
#         time.sleep(1.0 / 240.0)

#     print("Episode finished, total reward:", total_reward)
#     dataset = env.get_episode_dataset()
#     if dataset is not None:
#         print("Logged steps:", dataset["actions"].shape[0])

#     # example tracking rollout towards the fixed goal
#     print("\n--- Tracking controller rollout ---")
#     obs = env.reset(random_start=True)
#     done = False
#     total_reward = 0.0
#     while not done:
#         state = env.get_state()
#         pos = state["position"]
#         quat = state["orientation_quat"]
#         yaw = p.getEulerFromQuaternion(quat)[2]
#         # ref is just "go straight to goal"
#         ref_pose = [env.goal_xy[0], env.goal_xy[1], 0.0]
#         action = env.tracking_controller(ref_pose, state=state)
#         obs, reward, done, info = env.step(action)
#         total_reward += reward
#         time.sleep(1.0 / 240.0)

#     print("Tracking episode finished, total reward:", total_reward)
#     env.disconnect()





import pybullet as p
import pybullet_data
import time
import numpy as np
import os


class RoughTerrainEnv:
    def __init__(
        self,
        terrain_npy_path="jezero_128x128_1m.npy",
        texture_path="jezero_texture_128x128.png",
        gui=True,
        max_steps=500,
        log_data=False,
        height_window_size=16,
    ):
        """
        Rough terrain rover environment with:
          - reset / step
          - state / observation
          - logging
          - simple tracking controller
        """

        self.gui = gui
        self.max_steps = max_steps
        self.log_data = log_data
        self.height_window_size = height_window_size

        # connect to PyBullet
        self.client_id = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -3.71)

        # terrain parameters
        self.terrain_size = 128
        self.pixel_size_m = 1.0096633268711
        self.terrain_scale_xy = self.pixel_size_m

        # load heightmap
        assert os.path.exists(terrain_npy_path), f"Missing {terrain_npy_path}"
        heights = np.load(terrain_npy_path).astype(np.float32)
        
        assert heights.shape == (self.terrain_size, self.terrain_size)
        self.heights = heights

        # vertical exaggeration if needed
        hmin, hmax = float(np.nanmin(self.heights)), float(np.nanmax(self.heights))
        hrange = hmax - hmin
        print(f"[heightmap] min={hmin:.3f} max={hmax:.3f} range={hrange:.3f}")
        if hrange < 0.5:
            print("[heightmap] small range; applying vertical exaggeration ×50")
            self.heights *= 50.0

        # recompute stats and z offset
        hmin_f, hmax_f = float(np.nanmin(self.heights)), float(np.nanmax(self.heights))
        self.z_offset = (hmin_f + hmax_f) / 2.0
        print(f"[heightmap] Final min={hmin_f:.3f} max={hmax_f:.3f}")
        print(f"[heightmap] Applying Z-offset to terrain body: {self.z_offset:.3f}")

        # flatten for pybullet
        height_data = self.heights.reshape(-1).tolist()

        # create terrain collision
        print("Creating collision shape...")
        terrain_shape = p.createCollisionShape(
            shapeType=p.GEOM_HEIGHTFIELD,
            meshScale=[self.terrain_scale_xy, self.terrain_scale_xy, 1.0],
            heightfieldData=height_data,
            heightfieldTextureScaling=self.terrain_size - 1,
            numHeightfieldRows=self.terrain_size,
            numHeightfieldColumns=self.terrain_size,
        )

        print("Creating terrain body...")
        self.terrain_body = p.createMultiBody(
            baseMass=0, baseCollisionShapeIndex=terrain_shape
        )

        p.resetBasePositionAndOrientation(
            self.terrain_body, [0, 0, self.z_offset], [0, 0, 0, 1]
        )
        p.changeDynamics(self.terrain_body, -1, lateralFriction=2.0)

        # texture
        if os.path.exists(texture_path):
            print("Loading Mars texture...")
            texture_id = p.loadTexture(texture_path)
            p.changeVisualShape(self.terrain_body, -1, textureUniqueId=texture_id)
        else:
            print(f"[warn] texture {texture_path} not found, using default color")

        # robot config
                # robot config (Husky)
        self.robot_id = None
        self.left_wheels = []
        self.right_wheels = []

        # action limits (for Husky, can tune later)
        self.max_throttle = 30.0    # forward speed scale
        self.max_steering = 1.0     # turning command
        self.turn_gain = 1.5        # how strong steering affects left/right wheels


        # goal (x, y) in world frame
        self.goal_xy = np.array([20.0, 0.0])

        # logging buffers
        self.episode_log = []
        self.step_count = 0

    # ------------------------------------------------------------------
    # helpers for height lookup and local patch
    # ------------------------------------------------------------------
    def world_to_grid(self, x, y):
        """Convert world (x,y) to heightmap indices (row, col)."""
        i = int(np.round(x / self.terrain_scale_xy + self.terrain_size / 2))
        j = int(np.round(y / self.terrain_scale_xy + self.terrain_size / 2))
        i = np.clip(i, 0, self.terrain_size - 1)
        j = np.clip(j, 0, self.terrain_size - 1)
        return j, i  # row, col

    def get_height_at(self, x, y):
        row, col = self.world_to_grid(x, y)
        return float(self.heights[row, col])

    def get_local_height_patch(self, x, y):
        """
        Returns a small height patch around (x,y) in robot-centric world coords.
        Shape: (window_size, window_size)
        """
        ws = self.height_window_size
        row_c, col_c = self.world_to_grid(x, y)
        half = ws // 2

        row_min = max(0, row_c - half)
        row_max = min(self.terrain_size, row_c + half)
        col_min = max(0, col_c - half)
        col_max = min(self.terrain_size, col_c + half)

        patch = np.zeros((ws, ws), dtype=np.float32)
        sub = self.heights[row_min:row_max, col_min:col_max]
        patch[
            (row_min - (row_c - half)) : (row_max - (row_c - half)),
            (col_min - (col_c - half)) : (col_max - (col_c - half)),
        ] = sub
        return patch

    # ------------------------------------------------------------------
    # core env API
    # ------------------------------------------------------------------
    def reset(self, random_start=True):
        """Reset robot pose and episode log. Returns initial observation."""
        if self.robot_id is not None:
            p.removeBody(self.robot_id)

        # choose start position
        if random_start:
            # small random patch near center
            start_x = np.random.uniform(-5.0, 5.0)
            start_y = np.random.uniform(-5.0, 5.0)
        else:
            start_x, start_y = 0.0, 0.0

        hz = self.get_height_at(start_x, start_y)
        start_z = hz + 0.8

        start_pos = [start_x, start_y, start_z]
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])

        print("Loading robot (Husky)...")
        self.robot_id = p.loadURDF(
            "husky/husky.urdf",
            start_pos,
            start_orientation,
            globalScaling=0.8,   # 1.2–1.5 works nicely
        )

        # Slight friction for chassis
        p.changeDynamics(self.robot_id, -1, lateralFriction=0.8)

        # Identify wheel joints and give them more grip
        self.left_wheels = []
        self.right_wheels = []

        num_joints = p.getNumJoints(self.robot_id)
        for j in range(num_joints):
            info = p.getJointInfo(self.robot_id, j)
            name = info[1].decode("utf-8")
            if "wheel" in name:
                if "left" in name:
                    self.left_wheels.append(j)
                elif "right" in name:
                    self.right_wheels.append(j)

                p.changeDynamics(
                    self.robot_id,
                    j,
                    lateralFriction=1.8,
                    rollingFriction=0.05,
                    spinningFriction=0.05,
                )

        # keep camera roughly on the robot / center
        p.resetDebugVisualizerCamera(
            cameraDistance=12.0,
            cameraYaw=45.0,
            cameraPitch=-25.0,
            cameraTargetPosition=[start_x, start_y, hz],
        )

        self.step_count = 0
        self.episode_log = []

        obs = self.get_observation()
        return obs

    def step(self, action):
        """
        Take one step in the environment.

        action: np.array([throttle, steering])

        Returns: obs, reward, done, info
        """

        throttle = float(np.clip(action[0], -self.max_throttle, self.max_throttle))
        steering = float(np.clip(action[1], -self.max_steering, self.max_steering))

        self._apply_action(throttle, steering)

        p.stepSimulation()
        self.step_count += 1

        obs_next = self.get_observation()
        state_next = self.get_state()

        # reward = negative distance to goal
        pos = state_next["position"]
        xy = np.array(pos[:2])
        dist_to_goal = np.linalg.norm(xy - self.goal_xy)
        reward = -dist_to_goal

        # termination conditions
        done = False
        info = {"dist_to_goal": dist_to_goal}

        # robot height vs terrain
        base_z = pos[2]
        ground_z = self.get_height_at(pos[0], pos[1])
        if base_z - ground_z > 3.0:  # big jump, probably flipped or flying
            done = True
            info["termination"] = "unstable_height"
        if dist_to_goal < 1.0:
            done = True
            info["termination"] = "goal_reached"
        if self.step_count >= self.max_steps:
            done = True
            info["termination"] = "max_steps"

        # simple logging
        if self.log_data:
            self.episode_log.append(
                {
                    "obs": obs_next,
                    "action": np.array([throttle, steering], dtype=np.float32),
                    "reward": reward,
                    "done": done,
                }
            )

        return obs_next, reward, done, info

    # ------------------------------------------------------------------
    # state and observation
    # ------------------------------------------------------------------
    def get_state(self):
        """Returns a dict with pose and body twist in world frame."""
        assert self.robot_id is not None, "Robot not loaded yet"
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        lin_vel, ang_vel = p.getBaseVelocity(self.robot_id)

        state = {
            "position": np.array(pos, dtype=np.float32),
            "orientation_quat": np.array(orn, dtype=np.float32),
            "linear_velocity": np.array(lin_vel, dtype=np.float32),
            "angular_velocity": np.array(ang_vel, dtype=np.float32),
        }
        return state

    def get_observation(self):
        """
        Observation used by the dynamics model:
          - gravity vector in body frame (tilt)
          - body twist in body frame (v in R^6)
          - local height map around the robot
        """
        st = self.get_state()
        pos = st["position"]
        quat = st["orientation_quat"]
        lin_vel_world = st["linear_velocity"]
        ang_vel_world = st["angular_velocity"]

        # rotation matrix from quaternion
        rot = np.array(p.getMatrixFromQuaternion(quat), dtype=np.float32).reshape(3, 3)

        g_world = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        g_body = rot.T @ g_world  # gravity vector in body frame

        v_body = rot.T @ lin_vel_world
        w_body = rot.T @ ang_vel_world

        height_patch = self.get_local_height_patch(pos[0], pos[1])

        obs = {
            "gravity_body": g_body,
            "v_body": v_body,
            "w_body": w_body,
            "height_patch": height_patch,
            "position_xy": pos[:2],
        }
        return obs

    # ------------------------------------------------------------------
    # actuation and simple tracking controller
    # ------------------------------------------------------------------

    def _apply_action(self, throttle, steering):
        """Skid-steer mapping for Husky.

        throttle: forward command (-max_throttle .. max_throttle)
        steering: turn command   (-max_steering .. max_steering)
        """

        # clip inputs
        v = float(np.clip(throttle, -self.max_throttle, self.max_throttle))
        w = float(np.clip(steering, -self.max_steering, self.max_steering))

        # convert to left/right wheel velocities
        v_left = v - self.turn_gain * w
        v_right = v + self.turn_gain * w

        # optional: clip again
        v_left = float(np.clip(v_left, -self.max_throttle, self.max_throttle))
        v_right = float(np.clip(v_right, -self.max_throttle, self.max_throttle))

        # motor power
        motor_force = 150.0   # you can bump this if you want more torque

        # apply to left wheels
        for j in self.left_wheels:
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=j,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=v_left,
                force=motor_force,
            )

        # apply to right wheels
        for j in self.right_wheels:
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=j,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=v_right,
                force=motor_force,
            )



    def tracking_controller(self, ref_pose, state=None):
        """
        Very simple tracking controller.

        ref_pose: [x_ref, y_ref, yaw_ref]
        state: if None, uses current state.

        Returns: np.array([throttle, steering])
        """
        if state is None:
            state = self.get_state()

        pos = state["position"]
        quat = state["orientation_quat"]
        yaw = p.getEulerFromQuaternion(quat)[2]

        x, y = pos[0], pos[1]
        x_ref, y_ref, yaw_ref = ref_pose

        # errors
        dx = x_ref - x
        dy = y_ref - y
        dist = np.hypot(dx, dy)
        desired_heading = np.arctan2(dy, dx)
        heading_error = desired_heading - yaw

        # wrap to [-pi, pi]
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

        # simple PD-like law
        k_v = 1.0
        k_yaw = 1.5

        throttle = np.clip(k_v * dist, -self.max_throttle, self.max_throttle)
        steering = np.clip(k_yaw * heading_error, -self.max_steering, self.max_steering)

        return np.array([throttle, steering], dtype=np.float32)

    # ------------------------------------------------------------------
    # dataset helpers
    # ------------------------------------------------------------------
    def get_episode_dataset(self):
        """
        Returns numpy arrays (obs, actions, rewards, dones) for the last episode.
        You can extend this to save to disk.
        """
        if not self.episode_log:
            return None

        actions = np.stack([step["action"] for step in self.episode_log], axis=0)
        rewards = np.array([step["reward"] for step in self.episode_log], dtype=np.float32)
        dones = np.array([step["done"] for step in self.episode_log], dtype=np.bool_)

        # for obs we just keep gravity/v_body/w_body and flatten the height patch
        gravity = np.stack(
            [step["obs"]["gravity_body"] for step in self.episode_log], axis=0
        )
        v_body = np.stack(
            [step["obs"]["v_body"] for step in self.episode_log], axis=0
        )
        w_body = np.stack(
            [step["obs"]["w_body"] for step in self.episode_log], axis=0
        )
        height_patches = np.stack(
            [step["obs"]["height_patch"] for step in self.episode_log], axis=0
        )

        dataset = {
            "gravity_body": gravity,
            "v_body": v_body,
            "w_body": w_body,
            "height_patches": height_patches,
            "actions": actions,
            "rewards": rewards,
            "dones": dones,
        }
        return dataset

    def disconnect(self):
        p.disconnect(self.client_id)


# ----------------------------------------------------------------------
# example usage: random rollout and one tracking rollout
# ----------------------------------------------------------------------
if __name__ == "__main__":
    env = RoughTerrainEnv(gui=True, log_data=True)

    print("--- Random policy rollout ---")
    obs = env.reset(random_start=True)
    done = False
    total_reward = 0.0

    while not done:
        # random actions
        action = np.array(
            [
                np.random.uniform(-env.max_throttle, env.max_throttle),
                np.random.uniform(-env.max_steering, env.max_steering),
            ],
            dtype=np.float32,
        )
        obs, reward, done, info = env.step(action)
        total_reward += reward
        time.sleep(1.0 / 240.0)

    print("Episode finished, total reward:", total_reward)
    dataset = env.get_episode_dataset()
    if dataset is not None:
        print("Logged steps:", dataset["actions"].shape[0])

    # example tracking rollout towards the fixed goal
    print("\n--- Tracking controller rollout ---")
    obs = env.reset(random_start=True)
    done = False
    total_reward = 0.0
    while not done:
        state = env.get_state()
        pos = state["position"]
        quat = state["orientation_quat"]
        yaw = p.getEulerFromQuaternion(quat)[2]
        # ref is just "go straight to goal"
        ref_pose = [env.goal_xy[0], env.goal_xy[1], 0.0]
        action = env.tracking_controller(ref_pose, state=state)
        obs, reward, done, info = env.step(action)
        total_reward += reward
        time.sleep(1.0 / 240.0)

    print("Tracking episode finished, total reward:", total_reward)
    env.disconnect()
