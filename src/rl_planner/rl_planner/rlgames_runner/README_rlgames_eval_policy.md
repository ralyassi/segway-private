# 🧠 RL-Games Policy Evaluation

This module provides a lightweight wrapper for loading and evaluating trained reinforcement learning (RL) models using the **[RL-Games](https://github.com/Denys88/rl_games)** framework.  
It is designed for inference-only use, particularly for **social navigation** policies that take human-related observations as input.

---

## 🚀 Overview

### `rlgames_eval_policy.py`

Defines a single high-level class:

#### **`RLModel`**
A PyTorch module that:
- Loads a trained RL checkpoint (A2C / PPO)
- Normalizes inputs using stored running mean/std
- Evaluates the policy in deterministic or stochastic mode
- Supports RNN-based networks (keeps hidden states)
- Outputs both the action and value predictions

---

## ⚙️ Features

- ✅ Automatic model loading from checkpoint  
- ✅ Handles running mean/std normalization  
- ✅ Supports RNN memory states  
- ✅ Inference with or without randomness  
- ✅ Designed for multi-objective RL or social navigation tasks  

---

## 🧩 Class Usage

```python
from rlgames_eval_policy import RLModel

# Initialize model
model = RLModel(
    checkpoint_path='checkpoint/last_SocialNavHumanPoseMORL_ep_450_rew_446.22223.pth'
)

# Prepare observation dict
obs_dict = {'obs': obs_tensor, 'pref_vec': pref_tensor}

# Get deterministic or stochastic action
action = model.get_action(obs_dict, deterministic=True)
```

---

## 🧠 Network Components

- **`SocialNavHumanPose`**  
  Imported from `network.socialnav_humanpose`.  
  Defines the architecture for social navigation with human-aware inputs.

- **`RunningMeanStd`**  
  Used for input/output normalization during evaluation.

---

## ⚙️ Requirements

- Python ≥ 3.8  
- PyTorch ≥ 2.0  
- Numpy  
- `network` and `running_mean_std` modules from your project.

---

## 🧩 Notes

- Actions are normalized to the range **[-1, 1]**.  
- The class supports restoring and maintaining **RNN hidden states** between inference steps for temporal consistency.  
- Use `.reset_rnn_state()` to clear the memory between episodes.

---

## 📂 Typical Directory Structure

```
project_root/
├── rlgames_eval_policy.py
├── network/
│   └── socialnav_humanpose.py
├── running_mean_std.py
└── checkpoint/
    └── last_SocialNavHumanPoseMORL_ep_450_rew_446.22223.pth
```

---

## 📄 License

MIT License © 2025 Rashid Alyassi
