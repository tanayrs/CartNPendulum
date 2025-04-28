import os
import signal
import time
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from hardware_env import HardwareBalancingEnv
from safety_monitor import SafetyMonitor

# Configuration parameters
config = {
    "pretrained_model": "saved_models/PPO_model_jia",
    "policy": "MlpPolicy",
    "total_timesteps": 50000,  # You can adjust this based on how much additional training you want
    "save_freq": 5000,
    "log_dir": "hardware_training_logs",
    "save_path": "hardware_trained_models",
    "learning_rate": 0.0003,  # Lower learning rate for fine-tuning
}

def setup_signal_handlers(env, model, safety_monitor=None):
    """Set up signal handlers for graceful shutdown"""
    def signal_handler(sig, frame):
        print("\nSaving model before exit...")
        model.save(f"{config['save_path']}/interrupted_model")
        if safety_monitor is not None:
            safety_monitor.stop()
        env.close()
        print("Cleanup complete. Exiting.")
        exit(0)
    signal.signal(signal.SIGINT, signal_handler)

def load_pretrained_model(model_path, env):
    """Load a pre-trained model and adapt it to the new environment"""
    print(f"Loading pre-trained model from {model_path}...")
    try:
        # First try normal loading
        return PPO.load(model_path, env=env)
    except (ValueError, KeyError) as e:
        print(f"Standard loading failed: {e}")
        print("Attempting to load with custom objects...")
        # If that fails, try with custom_objects to handle differences
        return PPO.load(
            model_path, 
            env=env, 
            custom_objects={
                "learning_rate": config["learning_rate"],
                "clip_range": lambda _: 0.2,
                "n_steps": 2048,
            }
        )

def main():
    print("Initializing hardware environment...")
    # Create hardware environment
    env = HardwareBalancingEnv(max_steps=500)
    
    # Create safety monitor thread
    safety_monitor = SafetyMonitor(env, threshold=0.3)
    safety_monitor.start()
    
    # Create directories if they don't exist
    if not os.path.exists(config["save_path"]):
        os.makedirs(config["save_path"])
    if not os.path.exists(config["log_dir"]):
        os.makedirs(config["log_dir"])

    try:
        # Load pre-trained model
        model = load_pretrained_model(config["pretrained_model"], env)
        
        # Set up callback for saving checkpoints
        checkpoint_callback = CheckpointCallback(
            save_freq=config["save_freq"],
            save_path=config["save_path"],
            name_prefix="hardware_model"
        )
        
        # Set up signal handlers for graceful shutdown
        setup_signal_handlers(env, model, safety_monitor)
        
        print("Starting training...")
        print("Press Ctrl+C to save and exit at any time")
        
        # Continue training with loaded weights
        model.learn(
            total_timesteps=config["total_timesteps"],
            callback=checkpoint_callback,
            reset_num_timesteps=False,  # Continue timestep counting from pre-trained model
            tb_log_name="hardware_training"
        )
        
        # Save final model
        model_save_path = f"{config['save_path']}/final_hardware_model"
        print(f"Training complete. Saving final model to {model_save_path}")
        model.save(model_save_path)
        
    except Exception as e:
        print(f"Error during training: {e}")
    
    finally:
        # Clean up
        print("Cleaning up...")
        safety_monitor.stop()
        time.sleep(0.5)  # Give safety monitor thread time to stop
        env.close()
        print("Done!")

if __name__ == "__main__":
    main()
