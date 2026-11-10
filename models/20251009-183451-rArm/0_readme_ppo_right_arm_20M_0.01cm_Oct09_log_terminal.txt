(g1_vibe) york@RAD6801-Precision-3660:~/Documents/00DGX2025/unitree_g1_vibes/RL-shenanigans$ python3 train_g1_arm_policy.py --total-steps 20000000 --algo ppo --num-envs 256 --headless --right-arm
Using cuda device
/home/york/miniforge3/envs/g1_vibe/lib/python3.10/site-packages/stable_baselines3/common/on_policy_algorithm.py:150: UserWarning: You are trying to run PPO on the GPU, but it is primarily intended to run on the CPU when not using a CNN policy (you are using ActorCriticPolicy which should be a MlpPolicy). See https://github.com/DLR-RM/stable-baselines3/issues/1245 for more info. You can pass `device='cpu'` or `export CUDA_VISIBLE_DEVICES=` to force using the CPU.Note: The model will train, but the GPU utilization will be poor and the training might take longer than on CPU.
  warnings.warn(
Training config:
{'algo': 'ppo',
 'checkpoint': 250000,
 'headless': True,
 'num_envs': 256,
 'resume_from': None,
 'right_arm': True,
 'show_envs': 1,
 'total_steps': 20000000}
Logging to runs/g1_arm_reach/20251009-183451-rArm
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
-------------------------------
| time/              |        |
|    fps             | 3417   |
|    iterations      | 1      |
|    time_elapsed    | 153    |
|    total_timesteps | 524288 |
-------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_250k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3208    |
|    iterations      | 1       |
|    time_elapsed    | 163     |
|    total_timesteps | 1048576 |
--------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_500k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3164    |
|    iterations      | 1       |
|    time_elapsed    | 165     |
|    total_timesteps | 1572864 |
--------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_750k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3241    |
|    iterations      | 1       |
|    time_elapsed    | 161     |
|    total_timesteps | 2097152 |
--------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_1000k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3279    |
|    iterations      | 1       |
|    time_elapsed    | 159     |
|    total_timesteps | 2621440 |
--------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_1250k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3313    |
|    iterations      | 1       |
|    time_elapsed    | 158     |
|    total_timesteps | 3145728 |
--------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_1500k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3307    |
|    iterations      | 1       |
|    time_elapsed    | 158     |
|    total_timesteps | 3670016 |
--------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_1750k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3301    |
|    iterations      | 1       |
|    time_elapsed    | 158     |
|    total_timesteps | 4194304 |
--------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_2000k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3352    |
|    iterations      | 1       |
|    time_elapsed    | 156     |
|    total_timesteps | 4718592 |
--------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_2250k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3349    |
|    iterations      | 1       |
|    time_elapsed    | 156     |
|    total_timesteps | 5242880 |
--------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_2500k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3363    |
|    iterations      | 1       |
|    time_elapsed    | 155     |
|    total_timesteps | 5767168 |
--------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_2750k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3400    |
|    iterations      | 1       |
|    time_elapsed    | 154     |
|    total_timesteps | 6291456 |
--------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_3000k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3396    |
|    iterations      | 1       |
|    time_elapsed    | 154     |
|    total_timesteps | 6815744 |
--------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_3250k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3332    |
|    iterations      | 1       |
|    time_elapsed    | 157     |
|    total_timesteps | 7340032 |
--------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_3500k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3194    |
|    iterations      | 1       |
|    time_elapsed    | 164     |
|    total_timesteps | 7864320 |
--------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_3750k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3182    |
|    iterations      | 1       |
|    time_elapsed    | 164     |
|    total_timesteps | 8388608 |
--------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_4000k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3227    |
|    iterations      | 1       |
|    time_elapsed    | 162     |
|    total_timesteps | 8912896 |
--------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_4250k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3209    |
|    iterations      | 1       |
|    time_elapsed    | 163     |
|    total_timesteps | 9437184 |
--------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_4500k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3188    |
|    iterations      | 1       |
|    time_elapsed    | 164     |
|    total_timesteps | 9961472 |
--------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_4750k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3188     |
|    iterations      | 1        |
|    time_elapsed    | 164      |
|    total_timesteps | 10485760 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_5000k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3175     |
|    iterations      | 1        |
|    time_elapsed    | 165      |
|    total_timesteps | 11010048 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_5250k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3173     |
|    iterations      | 1        |
|    time_elapsed    | 165      |
|    total_timesteps | 11534336 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_5500k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3191     |
|    iterations      | 1        |
|    time_elapsed    | 164      |
|    total_timesteps | 12058624 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_5750k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3192     |
|    iterations      | 1        |
|    time_elapsed    | 164      |
|    total_timesteps | 12582912 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_6000k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3163     |
|    iterations      | 1        |
|    time_elapsed    | 165      |
|    total_timesteps | 13107200 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_6250k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3183     |
|    iterations      | 1        |
|    time_elapsed    | 164      |
|    total_timesteps | 13631488 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_6500k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3195     |
|    iterations      | 1        |
|    time_elapsed    | 164      |
|    total_timesteps | 14155776 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_6750k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3199     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 14680064 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_7000k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3214     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 15204352 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_7250k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3223     |
|    iterations      | 1        |
|    time_elapsed    | 162      |
|    total_timesteps | 15728640 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_7500k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3228     |
|    iterations      | 1        |
|    time_elapsed    | 162      |
|    total_timesteps | 16252928 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_7750k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3186     |
|    iterations      | 1        |
|    time_elapsed    | 164      |
|    total_timesteps | 16777216 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_8000k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3210     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 17301504 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_8250k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3209     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 17825792 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_8500k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3222     |
|    iterations      | 1        |
|    time_elapsed    | 162      |
|    total_timesteps | 18350080 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_8750k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3219     |
|    iterations      | 1        |
|    time_elapsed    | 162      |
|    total_timesteps | 18874368 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_9000k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3193     |
|    iterations      | 1        |
|    time_elapsed    | 164      |
|    total_timesteps | 19398656 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_9250k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3205     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 19922944 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_9500k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3208     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 20447232 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_9750k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3199     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 20971520 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_10000k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3208     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 21495808 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_10250k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3209     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 22020096 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_10500k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3212     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 22544384 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_10750k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3219     |
|    iterations      | 1        |
|    time_elapsed    | 162      |
|    total_timesteps | 23068672 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_11000k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3227     |
|    iterations      | 1        |
|    time_elapsed    | 162      |
|    total_timesteps | 23592960 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_11250k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3215     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 24117248 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_11500k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3222     |
|    iterations      | 1        |
|    time_elapsed    | 162      |
|    total_timesteps | 24641536 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_11750k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3233     |
|    iterations      | 1        |
|    time_elapsed    | 162      |
|    total_timesteps | 25165824 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_12000k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3214     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 25690112 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_12250k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3193     |
|    iterations      | 1        |
|    time_elapsed    | 164      |
|    total_timesteps | 26214400 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_12500k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3200     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 26738688 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_12750k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3176     |
|    iterations      | 1        |
|    time_elapsed    | 165      |
|    total_timesteps | 27262976 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_13000k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3204     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 27787264 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_13250k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3189     |
|    iterations      | 1        |
|    time_elapsed    | 164      |
|    total_timesteps | 28311552 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_13500k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3197     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 28835840 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_13750k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3201     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 29360128 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_14000k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3199     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 29884416 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_14250k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3217     |
|    iterations      | 1        |
|    time_elapsed    | 162      |
|    total_timesteps | 30408704 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_14500k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3192     |
|    iterations      | 1        |
|    time_elapsed    | 164      |
|    total_timesteps | 30932992 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_14750k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3216     |
|    iterations      | 1        |
|    time_elapsed    | 162      |
|    total_timesteps | 31457280 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_15000k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3201     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 31981568 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_15250k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3198     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 32505856 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_15500k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3216     |
|    iterations      | 1        |
|    time_elapsed    | 162      |
|    total_timesteps | 33030144 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_15750k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3186     |
|    iterations      | 1        |
|    time_elapsed    | 164      |
|    total_timesteps | 33554432 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_16000k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3186     |
|    iterations      | 1        |
|    time_elapsed    | 164      |
|    total_timesteps | 34078720 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_16250k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3212     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 34603008 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_16500k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3188     |
|    iterations      | 1        |
|    time_elapsed    | 164      |
|    total_timesteps | 35127296 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_16750k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3171     |
|    iterations      | 1        |
|    time_elapsed    | 165      |
|    total_timesteps | 35651584 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_17000k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3202     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 36175872 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_17250k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3184     |
|    iterations      | 1        |
|    time_elapsed    | 164      |
|    total_timesteps | 36700160 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_17500k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3196     |
|    iterations      | 1        |
|    time_elapsed    | 164      |
|    total_timesteps | 37224448 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_17750k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3184     |
|    iterations      | 1        |
|    time_elapsed    | 164      |
|    total_timesteps | 37748736 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_18000k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3210     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 38273024 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_18250k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3195     |
|    iterations      | 1        |
|    time_elapsed    | 164      |
|    total_timesteps | 38797312 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_18500k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3196     |
|    iterations      | 1        |
|    time_elapsed    | 164      |
|    total_timesteps | 39321600 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_18750k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3215     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 39845888 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_19000k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3195     |
|    iterations      | 1        |
|    time_elapsed    | 164      |
|    total_timesteps | 40370176 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_19250k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3208     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 40894464 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_19500k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3203     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 41418752 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_19750k.zip
Logging to runs/g1_arm_reach/20251009-183451-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3188     |
|    iterations      | 1        |
|    time_elapsed    | 164      |
|    total_timesteps | 41943040 |
---------------------------------
Checkpoint saved → models/20251009-183451-rArm/ppo_g1_right_20000k.zip
