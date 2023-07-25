import gym
from gym import error, spaces, utils
from gym.utils import seeding
import numpy as np
import matplotlib.pyplot as plt
import copy
import time
import scipy.io as io
import os


class SpatialTemporalFieldEnv(gym.Env):

    def __init__(
            self,
            learning_experiment_name,
            output_dir,
            field_size=[100, 100],
            max_num_steps=300,
            view_scope_half_side=5,
            adv_diff_params={}):
        # Open AI Gym stuff
        metadata = {'render.modes': ['human']}
        super(SpatialTemporalFieldEnv, self).__init__()

        # Set advection diffusion parameters
        self.dx = adv_diff_params.get("dx", 0.8)
        self.dy = adv_diff_params.get("dy", 0.8)
        self.vx = adv_diff_params.get("vx", -0.6)
        self.vy = adv_diff_params.get("vy", 0.8)
        self.dt = adv_diff_params.get("dt", 0.1)
        self.k = adv_diff_params.get("k", 1.0)

        print("Advection Diffusion parameters: ")
        print("dx, dy, vx, vy, dt, k: " +
              str((self.dx, self.dy, self.vx, self.vy, self.dt, self.k)))

        # Set field grid params/variables
        self.field_size = field_size
        self.field_area = self.field_size[0] * self.field_size[1]
        self.max_num_steps = max_num_steps
        self.view_scope_half_side = view_scope_half_side

        # Environment's field params
        self.env_curr_field = self.create_field()
        self.env_prev_field = np.zeros(self.field_size)

        # Agent Field related params/variables
        self.num_steps = 0
        self.agent_start_position = None
        self.agent_position = None
        self.agent_curr_field = np.zeros(self.field_size)
        self.agent_field_visited = np.zeros(self.field_size)
        self.agent_trajectory = []
        self.curr_view_scope = np.zeros(
            [2 * self.view_scope_half_side + 1, 2 * self.view_scope_half_side + 1])

        # Statistics variables
        self.agent_coverage = []
        self.rewards = []
        self.mapping_errors = []

        # Environment Observation space (Currently only has location as state)
        # Later, change it to include other states as well, such as gradient, concentration
        # And gradient
        self.observation_space = spaces.Box(
            low=0.0, high=99.0, shape=(2,), dtype=np.float32)

        # Agent Action related params/variables
        self.action_space_map = {}
        self.actions = ["left", "right", "up", "down"]
        self.action_space = spaces.Discrete(4)
        for action_id, action in enumerate(self.actions):
            self.action_space_map[action_id] = action

        # Misc. params
        self.output_dir = output_dir
        self.learning_experiment_name = learning_experiment_name

        # Make output_dir/learning_experiment_name folder if it doesn't exist already
        self.path_to_output_dir = os.path.join(
            self.output_dir, self.learning_experiment_name)
        if not os.path.exists(self.path_to_output_dir):
            os.makedirs(self.path_to_output_dir)
            print("Output directory " + self.path_to_output_dir + " created.")
        else:
            print("Output directory " +
                  self.path_to_output_dir + " already existed!")

        # Initial mapping error
        self.init_mapping_error = np.sum(self.env_curr_field)

    def create_field(self):
        cwd = os.getcwd()
        loaded_mat = io.loadmat(cwd + "/u.mat")
        u = loaded_mat.get('u')

        u_1 = u.copy()
        source_1_c_shift = 13

        for r in range(0, self.field_size[0]):
            for c in range(0, self.field_size[1]):
                u_1[r, c] = u[r % self.field_size[0],
                              (c - source_1_c_shift) % self.field_size[1]]

        reverse_u = u.copy()

        for r in range(0, self.field_size[0]):
            for c in range(0, self.field_size[1]):
                reverse_u[r, c] = u[self.field_size[0] - r - 1,
                                    self.field_size[1] - c - 1]

        reverse_u_1 = u.copy()
        source_2_r_shift = 10
        source_2_c_shift = 2

        # fmt: off
        for r in range(0, self.field_size[0]):
            for c in range(0, self.field_size[1]):
                reverse_u_1[r, c] = reverse_u[(r + source_2_r_shift) % self.field_size[0],
                                              (c + source_2_c_shift) % self.field_size[1]]
        # fmt: on
        combined_field = u_1 + reverse_u_1
        return combined_field

    def update_env_field(self):
        updated_u = self.env_curr_field.copy()
        u_k = self.env_curr_field.copy()

        dx = self.dx
        dy = self.dy
        dt = self.dt
        vx = self.vx
        vy = self.vy
        k = self.k

        # fmt: off
        for i in range(1, self.field_size[0] - 1):
            for j in range(1, self.field_size[1] - 1):
                updated_u[j, i] = u_k[j, i] + k * (dt / dx ** 2) * \
                    ((u_k[j + 1, i] + u_k[j - 1, i] +
                      u_k[j, i + 1] + u_k[j, i - 1] - 4 * u_k[j, i])) + \
                    vx * (dt / dx) * ((u_k[j + 1, i] - u_k[j, i])) + vy * (dt / dy) * \
                    (u_k[j, i + 1] - u_k[j, i])
        # fmt: on                                                                                                                                                                                          i])

        # self.env_prev_field = self.env_curr_field
        self.env_curr_field = updated_u

    def step(self, action_id):
        # Ensure action is a valid action and exists in Agent's action space
        assert self.action_space.contains(
            action_id), "Action %r (%s) is invalid!" % (action_id, type(action_id))

        action = self.action_space_map[action_id]
        assert action in ["left", "right", "up",
                          "down"], "%s (%s) invalid" % (action, type(action))

        # Get the next state
        (hit_wall, next_state) = self.get_next_state(action)
        if (hit_wall):
            # Terminate the episode, and return a large negative reward
            reward = -10
            self.rewards.append(reward)
            return (reward, self.agent_position, True, {})

        # Update field state
        self.update_env_field()

        # Update agent's view of the field
        frac_coverage_improvement = self.update_agent_field_and_coverage(
            next_state)

        # Update Mapping error
        curr_mapping_error = self.calculate_mapping_error()
        self.mapping_errors.append(curr_mapping_error)

        # Get the new reward
        reward = self.calculate_reward_2(
            next_state, frac_coverage_improvement)

        # Update number of steps
        self.num_steps += 1

        # Check for termination criteria
        done = False
        if (self.num_steps >= self.max_num_steps):
            done = True
            reward += 10

        self.rewards.append(reward)

        # Get any observations
        observations = {}

        # Update agent variables
        self.agent_position = next_state
        self.agent_trajectory.append(self.agent_position)

        # Return reward, next_state, done, observations
        return (reward, next_state, done, observations)

    def reset(self):
        # Reset agent related params
        self.num_steps = 0
        self.agent_position = self.choose_random_start_position()
        self.agent_curr_field = np.zeros(self.field_size)
        self.agent_field_visited = np.zeros(self.field_size)
        self.agent_trajectory = []
        self.curr_view_scope = np.zeros(
            [2 * self.view_scope_half_side + 1, 2 * self.view_scope_half_side + 1])

        # Reset environment related params
        self.env_curr_field = self.create_field()
        self.env_prev_field = np.zeros(self.field_size)

        # Reset stats
        self.agent_coverage = []
        self.rewards = []
        self.mapping_errors = []

        # Return the first state
        return self.agent_position

    def choose_random_start_position(self):
        return [np.random.randint(self.view_scope_half_side + 1, self.field_size[0] - self.view_scope_half_side - 1),
                np.random.randint(self.view_scope_half_side + 1, self.field_size[1] - self.view_scope_half_side - 1)]

    def get_next_state(self, action):
        # Create a deepcopy of current state
        next_state = copy.deepcopy(self.agent_position)
        if action == "left":
            next_state[1] = next_state[1] - 1
        elif action == "right":
            next_state[1] = next_state[1] + 1
        elif action == "up":
            next_state[0] = next_state[0] - 1
        elif action == "down":
            next_state[0] = next_state[0] + 1

        # Check for collisions
        hit_wall = False
        if ((next_state[0] < (0 + self.view_scope_half_side) or
             next_state[0] >= (self.field_size[0] - self.view_scope_half_side)) or
            ((next_state[1] < (0 + self.view_scope_half_side) or
              next_state[1] >= (self.field_size[1] - self.view_scope_half_side)))):
            # If the view scope is out of the field, hit_wall is set to True
            hit_wall = True

        return (hit_wall, next_state)

    def calculate_reward_1(self, next_state, frac_coverage_improvement):
        prev_mapping_error = 0
        if len(self.mapping_errors) != 0:
            prev_mapping_error = self.mapping_errors[-1]
        else:
            prev_mapping_error = self.init_mapping_error

        curr_mapping_error = self.calculate_mapping_error()

        self.mapping_errors.append(curr_mapping_error)

        mapping_error_improvement = prev_mapping_error - curr_mapping_error

        reward = (0.0001 * mapping_error_improvement) + \
            (2 * frac_coverage_improvement)
        return reward

    def calculate_reward_2(self, next_state, frac_coverage_improvement):
        """
        Assuming that the reward is only proportional to what is being copied by the viewscope.
        Coverage not considered.
        """
        # vs_val = np.sum(self.curr_view_scope)
        # if vs_val >= 1e-5:
        #     reward = -1000 * (1.0 / vs_val)
        # else:
        #     reward = -1000
        # return reward
        return 1e-2 * np.sum(self.curr_view_scope)

    def normalize(self, field):
        max_val = field.max()
        min_val = field.min()
        field_normalized = (field - min_val) / (max_val - min_val)
        return field_normalized

    def calculate_mapping_error(self):
        return np.sum(np.abs(self.agent_curr_field - self.env_curr_field))

    def update_agent_field_and_coverage(self, next_state):
        vs_min_row = next_state[0] - self.view_scope_half_side
        vs_max_row = next_state[0] + self.view_scope_half_side + 1

        vs_min_col = next_state[1] - self.view_scope_half_side
        vs_max_col = next_state[1] + self.view_scope_half_side + 1

        # Count prev_visited
        prev_visited = np.count_nonzero(self.agent_field_visited)

        # self.curr_view_scope = np.zeros(
        #     [vs_max_row - vs_min_row, vs_max_col - vs_min_col])

        for r in range(vs_min_row, vs_max_row):
            for c in range(vs_min_col, vs_max_col):
                self.agent_curr_field[r, c] = self.env_curr_field[r, c]
                self.agent_field_visited[r, c] = 1

        self.curr_view_scope = self.agent_curr_field[vs_min_row:vs_max_row,
                                                     vs_min_col:vs_max_col]

        # print("View Scope size: " + str(self.curr_view_scope.shape))

        # Count curr_visited
        curr_visited = np.count_nonzero(self.agent_field_visited)
        frac_coverage_improvement = float(
            curr_visited) - float(prev_visited) / float(self.field_area)

        # Record coverage percentage
        self.agent_coverage.append(
            (float(curr_visited) * 100.0) / float(self.field_area))
        return frac_coverage_improvement

    def save_episode_state(self, episode_num):
        fig_learning, fig_learning_axes = plt.subplots(2, 3, figsize=(15, 10))
        fig_learning_axes[0, 0].set_title("Environment Field End State")
        fig_learning_axes[0, 0].set_aspect("equal")

        fig_learning_axes[0, 1].set_title("Agent Field End State")
        fig_learning_axes[0, 1].set_aspect("equal")

        fig_learning_axes[0, 2].set_title("States visited")
        fig_learning_axes[0, 2].set_aspect("equal")

        fig_learning_axes[1, 0].set_title("Reward")
        # fig_learning_axes[1, 0].set_aspect("equal", adjustable='box')
        fig_learning_axes[1, 0].set_xlim([0, self.max_num_steps])

        fig_learning_axes[1, 1].set_title("Mapping Error")
        # fig_learning_axes[1, 1].set_aspect("equal", adjustable='box')
        fig_learning_axes[1, 1].set_xlim([0, self.max_num_steps])

        fig_learning_axes[1, 2].set_title("Field Coverage")
        # fig_learning_axes[1, 2].set_aspect("equal", adjustable='box')
        fig_learning_axes[1, 2].set_xlim([0, self.max_num_steps])
        fig_learning_axes[1, 2].set_ylim([0, 100])

        # Plot 1: Environment End state
        fig_learning_axes[0, 0].imshow(
            self.env_curr_field.T, cmap="Blues")

        traj_r = [position[0] for position in self.agent_trajectory]
        traj_c = [position[1] for position in self.agent_trajectory]
        fig_learning_axes[0, 0].plot(traj_r, traj_c, '.', color='black')

        fig_learning_axes[0, 0].plot(
            self.agent_trajectory[0][0], self.agent_trajectory[0][1], '*', color='red')

        # Plot 2: Agent Field End state
        fig_learning_axes[0, 1].imshow(
            self.agent_curr_field.T, cmap="Blues")

        fig_learning_axes[0, 1].plot(traj_r, traj_c, '.', color='black')

        fig_learning_axes[0, 1].plot(
            self.agent_trajectory[0][0], self.agent_trajectory[0][1], '*', color='red')

        # Plot 3: Visited
        fig_learning_axes[0, 2].imshow(
            self.agent_field_visited.T, cmap="Blues")

        fig_learning_axes[0, 2].plot(traj_r, traj_c, '.', color='black')

        fig_learning_axes[0, 2].plot(
            self.agent_trajectory[0][0], self.agent_trajectory[0][1], '*', color='red')

        # Plot 4: Reward
        fig_learning_axes[1, 0].plot(self.rewards, '.-')

        # Plot 5: Mapping Error
        fig_learning_axes[1, 1].plot(self.mapping_errors, '.-')

        # Plot 6: Coverage percentage
        fig_learning_axes[1, 2].plot(self.agent_coverage, '.-')

        # Add Episode number to top of image
        fig_learning.suptitle(
            "Episode number: " + str(episode_num) + ", Num timesteps: " + str(self.num_steps))

        # Save image to directory
        fig_file_name = "episode_" + str(episode_num) + ".png"
        plt.savefig(os.path.join(self.path_to_output_dir, fig_file_name))

        plt.close()

    def test_update_field_in_loop(self):
        fig = plt.figure(figsize=(8, 8))
        plt.ion()
        plt.show()
        self.agent_position = [7, 7]
        for i in range(100):
            plt.title("Step: " + str(i))

            plt.imshow(self.env_curr_field.T, cmap="Blues")
            traj_r = [position[0] for position in self.agent_trajectory]
            traj_c = [position[1] for position in self.agent_trajectory]
            plt.plot(traj_r, traj_c, '.', color='black')

            if (i % 2 == 0):
                (_, next_state) = self.get_next_state("right")
                self.agent_position = next_state
            else:
                (_, next_state) = self.get_next_state("down")
                self.agent_position = next_state

            self.agent_trajectory.append(next_state)

            plt.draw()
            plt.pause(0.001)
            self.update_env_field()
        plt.ioff()
        plt.close()

    def render(self, mode="human"):
        pass
