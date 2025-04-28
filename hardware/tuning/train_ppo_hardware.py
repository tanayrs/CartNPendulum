# File 3: train_ppo_hardware.py
import os
import argparse
from ppo_trainer import RealWorldPPOTrainer

def main():
    parser = argparse.ArgumentParser(description='Real-World PPO Training')
    parser.add_argument('--load-sim', type=str, help='Path to simulation model')
    parser.add_argument('--output-dir', type=str, default='realworld_models')
    args = parser.parse_args()

    env_config = {
        "max_angle": 15.0,
        "max_position": 0.5,
        "max_episode_steps": 500,
        "safety_threshold": 25.0
    }
    
    trainer = RealWorldPPOTrainer(
        env_config=env_config,
        learning_rate=1e-4,
        n_steps=512,
        batch_size=32,
        clip_range=0.1,
        gamma=0.98
    )
    
    if args.load_sim:
        print(f"Loading simulation model: {args.load_sim}")
        trainer.load_model(args.load_sim)

    try:
        print("=== Phase 1: Safe Exploration ===")
        trainer.train(total_timesteps=5000)
        trainer.save_model(os.path.join(args.output_dir, "phase1"))
        
        print("=== Phase 2: Policy Refinement ===")
        trainer.train(total_timesteps=10000)
        trainer.save_model(os.path.join(args.output_dir, "phase2"))
        
        print("=== Final Training Phase ===")
        trainer.train(total_timesteps=20000)
        trainer.save_model(os.path.join(args.output_dir, "final_model"))

    except KeyboardInterrupt:
        print("Training interrupted - saving current model...")
        trainer.save_model(os.path.join(args.output_dir, "interrupted_model"))
        
if __name__ == "__main__":
    main()

