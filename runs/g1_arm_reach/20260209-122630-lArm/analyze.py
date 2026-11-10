from tensorboard.backend.event_processing import event_accumulator
import pandas as pd

# 加载TensorBoard数据
ea = event_accumulator.EventAccumulator('runs/g1_arm_reach/20260209-122630-lArm/RL_0')
ea.Reload()

print("可用的标量标签：", ea.Tags()['scalars'])