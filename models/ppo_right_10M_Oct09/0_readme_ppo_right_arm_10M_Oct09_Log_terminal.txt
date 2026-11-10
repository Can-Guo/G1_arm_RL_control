(g1_vibe) york@RAD6801-Precision-3660:~/Documents/00DGX2025/unitree_g1_vibes/RL-shenanigans$ python3 train_g1_arm_policy.py --total-steps 10000000 --algo ppo --num-envs 256 --headless --right-arm
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
 'total_steps': 10000000}
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
-------------------------------
| time/              |        |
|    fps             | 3355   |
|    iterations      | 1      |
|    time_elapsed    | 156    |
|    total_timesteps | 524288 |
-------------------------------
Checkpoint saved → models/ppo_g1_right_250k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3159    |
|    iterations      | 1       |
|    time_elapsed    | 165     |
|    total_timesteps | 1048576 |
--------------------------------
Checkpoint saved → models/ppo_g1_right_500k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3206    |
|    iterations      | 1       |
|    time_elapsed    | 163     |
|    total_timesteps | 1572864 |
--------------------------------
Checkpoint saved → models/ppo_g1_right_750k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3225    |
|    iterations      | 1       |
|    time_elapsed    | 162     |
|    total_timesteps | 2097152 |
--------------------------------
Checkpoint saved → models/ppo_g1_right_1000k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3244    |
|    iterations      | 1       |
|    time_elapsed    | 161     |
|    total_timesteps | 2621440 |
--------------------------------
Checkpoint saved → models/ppo_g1_right_1250k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3273    |
|    iterations      | 1       |
|    time_elapsed    | 160     |
|    total_timesteps | 3145728 |
--------------------------------
Checkpoint saved → models/ppo_g1_right_1500k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3274    |
|    iterations      | 1       |
|    time_elapsed    | 160     |
|    total_timesteps | 3670016 |
--------------------------------
Checkpoint saved → models/ppo_g1_right_1750k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3309    |
|    iterations      | 1       |
|    time_elapsed    | 158     |
|    total_timesteps | 4194304 |
--------------------------------
Checkpoint saved → models/ppo_g1_right_2000k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3262    |
|    iterations      | 1       |
|    time_elapsed    | 160     |
|    total_timesteps | 4718592 |
--------------------------------
Checkpoint saved → models/ppo_g1_right_2250k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3225    |
|    iterations      | 1       |
|    time_elapsed    | 162     |
|    total_timesteps | 5242880 |
--------------------------------
Checkpoint saved → models/ppo_g1_right_2500k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3214    |
|    iterations      | 1       |
|    time_elapsed    | 163     |
|    total_timesteps | 5767168 |
--------------------------------
Checkpoint saved → models/ppo_g1_right_2750k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3233    |
|    iterations      | 1       |
|    time_elapsed    | 162     |
|    total_timesteps | 6291456 |
--------------------------------
Checkpoint saved → models/ppo_g1_right_3000k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3234    |
|    iterations      | 1       |
|    time_elapsed    | 162     |
|    total_timesteps | 6815744 |
--------------------------------
Checkpoint saved → models/ppo_g1_right_3250k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3275    |
|    iterations      | 1       |
|    time_elapsed    | 160     |
|    total_timesteps | 7340032 |
--------------------------------
Checkpoint saved → models/ppo_g1_right_3500k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3267    |
|    iterations      | 1       |
|    time_elapsed    | 160     |
|    total_timesteps | 7864320 |
--------------------------------
Checkpoint saved → models/ppo_g1_right_3750k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3302    |
|    iterations      | 1       |
|    time_elapsed    | 158     |
|    total_timesteps | 8388608 |
--------------------------------
Checkpoint saved → models/ppo_g1_right_4000k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3304    |
|    iterations      | 1       |
|    time_elapsed    | 158     |
|    total_timesteps | 8912896 |
--------------------------------
Checkpoint saved → models/ppo_g1_right_4250k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3314    |
|    iterations      | 1       |
|    time_elapsed    | 158     |
|    total_timesteps | 9437184 |
--------------------------------
Checkpoint saved → models/ppo_g1_right_4500k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3389    |
|    iterations      | 1       |
|    time_elapsed    | 154     |
|    total_timesteps | 9961472 |
--------------------------------
Checkpoint saved → models/ppo_g1_right_4750k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3444     |
|    iterations      | 1        |
|    time_elapsed    | 152      |
|    total_timesteps | 10485760 |
---------------------------------
Checkpoint saved → models/ppo_g1_right_5000k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3449     |
|    iterations      | 1        |
|    time_elapsed    | 151      |
|    total_timesteps | 11010048 |
---------------------------------
Checkpoint saved → models/ppo_g1_right_5250k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3432     |
|    iterations      | 1        |
|    time_elapsed    | 152      |
|    total_timesteps | 11534336 |
---------------------------------
Checkpoint saved → models/ppo_g1_right_5500k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3427     |
|    iterations      | 1        |
|    time_elapsed    | 152      |
|    total_timesteps | 12058624 |
---------------------------------
Checkpoint saved → models/ppo_g1_right_5750k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3363     |
|    iterations      | 1        |
|    time_elapsed    | 155      |
|    total_timesteps | 12582912 |
---------------------------------
Checkpoint saved → models/ppo_g1_right_6000k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3402     |
|    iterations      | 1        |
|    time_elapsed    | 154      |
|    total_timesteps | 13107200 |
---------------------------------
Checkpoint saved → models/ppo_g1_right_6250k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 2470     |
|    iterations      | 1        |
|    time_elapsed    | 212      |
|    total_timesteps | 13631488 |
---------------------------------
Checkpoint saved → models/ppo_g1_right_6500k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3165     |
|    iterations      | 1        |
|    time_elapsed    | 165      |
|    total_timesteps | 14155776 |
---------------------------------
Checkpoint saved → models/ppo_g1_right_6750k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3319     |
|    iterations      | 1        |
|    time_elapsed    | 157      |
|    total_timesteps | 14680064 |
---------------------------------
Checkpoint saved → models/ppo_g1_right_7000k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3318     |
|    iterations      | 1        |
|    time_elapsed    | 157      |
|    total_timesteps | 15204352 |
---------------------------------
Checkpoint saved → models/ppo_g1_right_7250k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3337     |
|    iterations      | 1        |
|    time_elapsed    | 157      |
|    total_timesteps | 15728640 |
---------------------------------
Checkpoint saved → models/ppo_g1_right_7500k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3360     |
|    iterations      | 1        |
|    time_elapsed    | 155      |
|    total_timesteps | 16252928 |
---------------------------------
Checkpoint saved → models/ppo_g1_right_7750k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3328     |
|    iterations      | 1        |
|    time_elapsed    | 157      |
|    total_timesteps | 16777216 |
---------------------------------
Checkpoint saved → models/ppo_g1_right_8000k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3356     |
|    iterations      | 1        |
|    time_elapsed    | 156      |
|    total_timesteps | 17301504 |
---------------------------------
Checkpoint saved → models/ppo_g1_right_8250k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3366     |
|    iterations      | 1        |
|    time_elapsed    | 155      |
|    total_timesteps | 17825792 |
---------------------------------
Checkpoint saved → models/ppo_g1_right_8500k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3349     |
|    iterations      | 1        |
|    time_elapsed    | 156      |
|    total_timesteps | 18350080 |
---------------------------------
Checkpoint saved → models/ppo_g1_right_8750k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3353     |
|    iterations      | 1        |
|    time_elapsed    | 156      |
|    total_timesteps | 18874368 |
---------------------------------
Checkpoint saved → models/ppo_g1_right_9000k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3370     |
|    iterations      | 1        |
|    time_elapsed    | 155      |
|    total_timesteps | 19398656 |
---------------------------------
Checkpoint saved → models/ppo_g1_right_9250k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3392     |
|    iterations      | 1        |
|    time_elapsed    | 154      |
|    total_timesteps | 19922944 |
---------------------------------
Checkpoint saved → models/ppo_g1_right_9500k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3370     |
|    iterations      | 1        |
|    time_elapsed    | 155      |
|    total_timesteps | 20447232 |
---------------------------------
Checkpoint saved → models/ppo_g1_right_9750k.zip
Logging to runs/g1_arm_reach/20251009-162035-rArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3354     |
|    iterations      | 1        |
|    time_elapsed    | 156      |
|    total_timesteps | 20971520 |
---------------------------------
Checkpoint saved → models/ppo_g1_right_10000k.zip
Final policy saved to models/ppo_g1_right_final.zip
