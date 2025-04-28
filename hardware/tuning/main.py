# File: main.py (updated)
# Add to imports
from controller_rl import RealWorldTrainingAgent

# Replace main loop with:
if __name__ == "__main__":
    env_config = {
        "max_angle": 15.0,
        "max_position": 0.5,
        "max_episode_steps": 500,
        "safety_threshold": 25.0
    }
    
    # Initialize agent with SAC for better sample efficiency
    agent = RealWorldTrainingAgent(
        model_type='SAC',
        policy='MlpPolicy',
        env_config=env_config,
        buffer_size=100000,
        learning_starts=1000,
        batch_size=256,
        tau=0.005,
        gamma=0.99
    )
    
    # Training phases
    try:
        # Phase 1: Initial collection
        print("=== Phase 1: Initial Experience Collection ===")
        agent.collect_experience(total_timesteps=5000)
        
        # Phase 2: Fine-tuning
        print("\n=== Phase 2: Fine-tuning ===")
        for epoch in range(10):
            print(f"Epoch {epoch+1}/10")
            agent.train(total_timesteps=1000)
            agent.collect_experience(total_timesteps=500)
            agent.save_model(f"sac_cartpole_{epoch+1}")
            
    except KeyboardInterrupt:
        pass
    finally:
        agent.env.close()
        agent.save_model("sac_cartpole_final")
        logger.stop()
