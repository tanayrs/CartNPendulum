import os
import signal
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from hardware_env import HardwareBalancingEnv

# Training parameters
config = {
    "policy": "MlpPolicy",
    "total_timesteps": 1e5,
    "save_freq": 10000,
    "log_dir": "hardware_training_logs",
    "save_path": "hardware_trained_models"
}

def setup_signal_handlers(env, model):
    def signal_handler(sig, frame):
        print("Saving model before exit...")
        model.save(f"{config['save_path']}/interrupted_model")
        env.close()
        exit(0)
    signal.signal(signal.SIGINT, signal_handler)

def main():
    # Create environment
    env = HardwareBalancingEnv(max_steps=500)
    
    # Initialize model
    model = PPO(
        config["policy"],
        env,
        verbose=1,
        device="cpu",
        batch_size=64,
        n_steps=256,
        tensorboard_log=config["log_dir"]
    )
    
    # Setup callbacks
    checkpoint_callback = CheckpointCallback(
        save_freq=config["save_freq"],
        save_path=config["save_path"],
        name_prefix="hardware_model"
    )
    
    # Setup safety handlers
    setup_signal_handlers(env, model)
    
    # Start training
    model.learn(
        total_timesteps=config["total_timesteps"],
        callback=checkpoint_callback,
        reset_num_timesteps=False
    )
    
    # Save final model
    model.save(f"{config['save_path']}/final_hardware_model")
    env.close()

if __name__ == "__main__":
    if not os.path.exists(config["save_path"]):
        os.makedirs(config["save_path"])
    main()

