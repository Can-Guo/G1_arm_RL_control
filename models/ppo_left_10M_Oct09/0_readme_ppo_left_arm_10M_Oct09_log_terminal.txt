(g1_vibe) york@RAD6801-Precision-3660:~/Documents/00DGX2025/unitree_g1_vibes/RL-shenanigans$ python3 train_g1_arm_policy.py --total-steps 10000000 --algo ppo --num-envs 256 --headless
Using cuda device
/home/york/miniforge3/envs/g1_vibe/lib/python3.10/site-packages/stable_baselines3/common/on_policy_algorithm.py:150: UserWarning: You are trying to run PPO on the GPU, but it is primarily intended to run on the CPU when not using a CNN policy (you are using ActorCriticPolicy which should be a MlpPolicy). See https://github.com/DLR-RM/stable-baselines3/issues/1245 for more info. You can pass `device='cpu'` or `export CUDA_VISIBLE_DEVICES=` to force using the CPU.Note: The model will train, but the GPU utilization will be poor and the training might take longer than on CPU.
  warnings.warn(
Training config:
{'algo': 'ppo',
 'checkpoint': 250000,
 'headless': True,
 'num_envs': 256,
 'resume_from': None,
 'right_arm': False,
 'show_envs': 1,
 'total_steps': 10000000}
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
-------------------------------
| time/              |        |
|    fps             | 3237   |
|    iterations      | 1      |
|    time_elapsed    | 161    |
|    total_timesteps | 524288 |
-------------------------------
Checkpoint saved → models/ppo_g1_left_250k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3134    |
|    iterations      | 1       |
|    time_elapsed    | 167     |
|    total_timesteps | 1048576 |
--------------------------------
Checkpoint saved → models/ppo_g1_left_500k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3118    |
|    iterations      | 1       |
|    time_elapsed    | 168     |
|    total_timesteps | 1572864 |
--------------------------------
Checkpoint saved → models/ppo_g1_left_750k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3078    |
|    iterations      | 1       |
|    time_elapsed    | 170     |
|    total_timesteps | 2097152 |
--------------------------------
Checkpoint saved → models/ppo_g1_left_1000k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3077    |
|    iterations      | 1       |
|    time_elapsed    | 170     |
|    total_timesteps | 2621440 |
--------------------------------
Checkpoint saved → models/ppo_g1_left_1250k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3055    |
|    iterations      | 1       |
|    time_elapsed    | 171     |
|    total_timesteps | 3145728 |
--------------------------------
Checkpoint saved → models/ppo_g1_left_1500k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3086    |
|    iterations      | 1       |
|    time_elapsed    | 169     |
|    total_timesteps | 3670016 |
--------------------------------
Checkpoint saved → models/ppo_g1_left_1750k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3094    |
|    iterations      | 1       |
|    time_elapsed    | 169     |
|    total_timesteps | 4194304 |
--------------------------------
Checkpoint saved → models/ppo_g1_left_2000k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3166    |
|    iterations      | 1       |
|    time_elapsed    | 165     |
|    total_timesteps | 4718592 |
--------------------------------
Checkpoint saved → models/ppo_g1_left_2250k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3180    |
|    iterations      | 1       |
|    time_elapsed    | 164     |
|    total_timesteps | 5242880 |
--------------------------------
Checkpoint saved → models/ppo_g1_left_2500k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3126    |
|    iterations      | 1       |
|    time_elapsed    | 167     |
|    total_timesteps | 5767168 |
--------------------------------
Checkpoint saved → models/ppo_g1_left_2750k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3172    |
|    iterations      | 1       |
|    time_elapsed    | 165     |
|    total_timesteps | 6291456 |
--------------------------------
Checkpoint saved → models/ppo_g1_left_3000k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3103    |
|    iterations      | 1       |
|    time_elapsed    | 168     |
|    total_timesteps | 6815744 |
--------------------------------
Checkpoint saved → models/ppo_g1_left_3250k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3256    |
|    iterations      | 1       |
|    time_elapsed    | 160     |
|    total_timesteps | 7340032 |
--------------------------------
Checkpoint saved → models/ppo_g1_left_3500k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3347    |
|    iterations      | 1       |
|    time_elapsed    | 156     |
|    total_timesteps | 7864320 |
--------------------------------
Checkpoint saved → models/ppo_g1_left_3750k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3353    |
|    iterations      | 1       |
|    time_elapsed    | 156     |
|    total_timesteps | 8388608 |
--------------------------------
Checkpoint saved → models/ppo_g1_left_4000k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3402    |
|    iterations      | 1       |
|    time_elapsed    | 154     |
|    total_timesteps | 8912896 |
--------------------------------
Checkpoint saved → models/ppo_g1_left_4250k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3393    |
|    iterations      | 1       |
|    time_elapsed    | 154     |
|    total_timesteps | 9437184 |
--------------------------------
Checkpoint saved → models/ppo_g1_left_4500k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
--------------------------------
| time/              |         |
|    fps             | 3382    |
|    iterations      | 1       |
|    time_elapsed    | 154     |
|    total_timesteps | 9961472 |
--------------------------------
Checkpoint saved → models/ppo_g1_left_4750k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3401     |
|    iterations      | 1        |
|    time_elapsed    | 154      |
|    total_timesteps | 10485760 |
---------------------------------
Checkpoint saved → models/ppo_g1_left_5000k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3374     |
|    iterations      | 1        |
|    time_elapsed    | 155      |
|    total_timesteps | 11010048 |
---------------------------------
Checkpoint saved → models/ppo_g1_left_5250k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3309     |
|    iterations      | 1        |
|    time_elapsed    | 158      |
|    total_timesteps | 11534336 |
---------------------------------
Checkpoint saved → models/ppo_g1_left_5500k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3308     |
|    iterations      | 1        |
|    time_elapsed    | 158      |
|    total_timesteps | 12058624 |
---------------------------------
Checkpoint saved → models/ppo_g1_left_5750k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3325     |
|    iterations      | 1        |
|    time_elapsed    | 157      |
|    total_timesteps | 12582912 |
---------------------------------
Checkpoint saved → models/ppo_g1_left_6000k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3322     |
|    iterations      | 1        |
|    time_elapsed    | 157      |
|    total_timesteps | 13107200 |
---------------------------------
Checkpoint saved → models/ppo_g1_left_6250k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3333     |
|    iterations      | 1        |
|    time_elapsed    | 157      |
|    total_timesteps | 13631488 |
---------------------------------
Checkpoint saved → models/ppo_g1_left_6500k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3322     |
|    iterations      | 1        |
|    time_elapsed    | 157      |
|    total_timesteps | 14155776 |
---------------------------------
Checkpoint saved → models/ppo_g1_left_6750k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3349     |
|    iterations      | 1        |
|    time_elapsed    | 156      |
|    total_timesteps | 14680064 |
---------------------------------
Checkpoint saved → models/ppo_g1_left_7000k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3308     |
|    iterations      | 1        |
|    time_elapsed    | 158      |
|    total_timesteps | 15204352 |
---------------------------------
Checkpoint saved → models/ppo_g1_left_7250k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3316     |
|    iterations      | 1        |
|    time_elapsed    | 158      |
|    total_timesteps | 15728640 |
---------------------------------
Checkpoint saved → models/ppo_g1_left_7500k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3328     |
|    iterations      | 1        |
|    time_elapsed    | 157      |
|    total_timesteps | 16252928 |
---------------------------------
Checkpoint saved → models/ppo_g1_left_7750k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3432     |
|    iterations      | 1        |
|    time_elapsed    | 152      |
|    total_timesteps | 16777216 |
---------------------------------
Checkpoint saved → models/ppo_g1_left_8000k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3429     |
|    iterations      | 1        |
|    time_elapsed    | 152      |
|    total_timesteps | 17301504 |
---------------------------------
Checkpoint saved → models/ppo_g1_left_8250k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3423     |
|    iterations      | 1        |
|    time_elapsed    | 153      |
|    total_timesteps | 17825792 |
---------------------------------
Checkpoint saved → models/ppo_g1_left_8500k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3428     |
|    iterations      | 1        |
|    time_elapsed    | 152      |
|    total_timesteps | 18350080 |
---------------------------------
Checkpoint saved → models/ppo_g1_left_8750k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3423     |
|    iterations      | 1        |
|    time_elapsed    | 153      |
|    total_timesteps | 18874368 |
---------------------------------
Checkpoint saved → models/ppo_g1_left_9000k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3418     |
|    iterations      | 1        |
|    time_elapsed    | 153      |
|    total_timesteps | 19398656 |
---------------------------------
Checkpoint saved → models/ppo_g1_left_9250k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
Killed
(g1_vibe) york@RAD6801-Precision-3660:~/Documents/00DGX2025/unitree_g1_vibes/RL-shenanigans$ python3 train_g1_arm_policy.py --total-steps 750000 --algo ppo --num-envs 256 --headless --resume-from /home/york/Documents/00DGX2025/unitree_g1_vibes/RL-shenanigans/models/ppo_g1_left_9250k.zip
Resuming training from '/home/york/Documents/00DGX2025/unitree_g1_vibes/RL-shenanigans/models/ppo_g1_left_9250k.zip' …
/home/york/miniforge3/envs/g1_vibe/lib/python3.10/site-packages/stable_baselines3/common/on_policy_algorithm.py:150: UserWarning: You are trying to run PPO on the GPU, but it is primarily intended to run on the CPU when not using a CNN policy (you are using ActorCriticPolicy which should be a MlpPolicy). See https://github.com/DLR-RM/stable-baselines3/issues/1245 for more info. You can pass `device='cpu'` or `export CUDA_VISIBLE_DEVICES=` to force using the CPU.Note: The model will train, but the GPU utilization will be poor and the training might take longer than on CPU.
  warnings.warn(
Training config:
{'algo': 'ppo',
 'checkpoint': 250000,
 'headless': True,
 'num_envs': 256,
 'resume_from': PosixPath('/home/york/Documents/00DGX2025/unitree_g1_vibes/RL-shenanigans/models/ppo_g1_left_9250k.zip'),
 'right_arm': False,
 'show_envs': 1,
 'total_steps': 750000}
Logging to runs/g1_arm_reach/20251010-094318-lArm
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3163     |
|    iterations      | 1        |
|    time_elapsed    | 165      |
|    total_timesteps | 19922944 |
---------------------------------
Checkpoint saved → models/20251010-094318-lArm/ppo_g1_left_19648k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3088     |
|    iterations      | 1        |
|    time_elapsed    | 169      |
|    total_timesteps | 20447232 |
---------------------------------
Checkpoint saved → models/20251010-094318-lArm/ppo_g1_left_19898k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3148     |
|    iterations      | 1        |
|    time_elapsed    | 166      |
|    total_timesteps | 20971520 |
---------------------------------
Checkpoint saved → models/20251010-094318-lArm/ppo_g1_left_20148k.zip
Final policy saved to models/20251010-094318-lArm/ppo_g1_left_final.zip
(g1_vibe) york@RAD6801-Precision-3660:~/Documents/00DGX2025/unitree_g1_vibes/RL-shenanigans$ python3 train_g1_arm_policy.py --total-steps 750000 --algo ppo --num-envs 256 --headless --resume-from /home/york/Documents/00DGX2025/unitree_g1_vibes/RL-shenanigans/models/ppo_g1_left_9250k.zip
Traceback (most recent call last):
  File "/home/york/Documents/00DGX2025/unitree_g1_vibes/RL-shenanigans/train_g1_arm_policy.py", line 176, in <module>
    main()
  File "/home/york/Documents/00DGX2025/unitree_g1_vibes/RL-shenanigans/train_g1_arm_policy.py", line 123, in main
    raise FileNotFoundError(f"Checkpoint not found: {args.resume_from}")
FileNotFoundError: Checkpoint not found: /home/york/Documents/00DGX2025/unitree_g1_vibes/RL-shenanigans/models/ppo_g1_left_9250k.zip
(g1_vibe) york@RAD6801-Precision-3660:~/Documents/00DGX2025/unitree_g1_vibes/RL-shenanigans$ python3 train_g1_arm_policy.py --total-steps 750000 --algo ppo --num-envs 256 --headless --resume-from /home/york/Documents/00DGX2025/unitree_g1_vibes/RL-shenanigans/models/ppo_left_10M_Oct09/ppo_g1_left_9250k.zip 
Resuming training from '/home/york/Documents/00DGX2025/unitree_g1_vibes/RL-shenanigans/models/ppo_left_10M_Oct09/ppo_g1_left_9250k.zip' …
/home/york/miniforge3/envs/g1_vibe/lib/python3.10/site-packages/stable_baselines3/common/on_policy_algorithm.py:150: UserWarning: You are trying to run PPO on the GPU, but it is primarily intended to run on the CPU when not using a CNN policy (you are using ActorCriticPolicy which should be a MlpPolicy). See https://github.com/DLR-RM/stable-baselines3/issues/1245 for more info. You can pass `device='cpu'` or `export CUDA_VISIBLE_DEVICES=` to force using the CPU.Note: The model will train, but the GPU utilization will be poor and the training might take longer than on CPU.
  warnings.warn(
Resuming from 19398656 steps
Training config:
{'algo': 'ppo',
 'checkpoint': 250000,
 'headless': True,
 'num_envs': 256,
 'resume_from': PosixPath('/home/york/Documents/00DGX2025/unitree_g1_vibes/RL-shenanigans/models/ppo_left_10M_Oct09/ppo_g1_left_9250k.zip'),
 'right_arm': False,
 'show_envs': 1,
 'total_steps': 750000}
Logging to runs/g1_arm_reach/20251010-100658-lArm
Training PPO policy for 20148656 steps …
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3202     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 19922944 |
---------------------------------
Saving checkpoint to models/20251010-100658-lArm/ppo_g1_left_19648k.zip .. with 19648656  steps
Checkpoint saved → models/20251010-100658-lArm/ppo_g1_left_19648k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3219     |
|    iterations      | 1        |
|    time_elapsed    | 162      |
|    total_timesteps | 20447232 |
---------------------------------
Saving checkpoint to models/20251010-100658-lArm/ppo_g1_left_19898k.zip .. with 19898656  steps
Checkpoint saved → models/20251010-100658-lArm/ppo_g1_left_19898k.zip
Logging to runs/g1_arm_reach/20251009-152632-lArm/RL_0
---------------------------------
| time/              |          |
|    fps             | 3202     |
|    iterations      | 1        |
|    time_elapsed    | 163      |
|    total_timesteps | 20971520 |
---------------------------------
Saving checkpoint to models/20251010-100658-lArm/ppo_g1_left_20148k.zip .. with 20148656  steps
Checkpoint saved → models/20251010-100658-lArm/ppo_g1_left_20148k.zip

