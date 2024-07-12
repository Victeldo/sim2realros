import numpy as np
import math
import robosuite as suite
from stable_baselines3 import SAC
from robosuite.wrappers import GymWrapper
from robosuite.controllers import load_controller_config

import rospy
from robosuite_publisher_pkg.msg import Array

controller_config = load_controller_config(default_controller="OSC_POSE")

# create environment instance
env = suite.make(
    env_name="Door", # try with other tasks like "Stack" and "Door"
    robots="Kinova3",  # try with other robots like "Sawyer" and "Jaco"
    has_renderer=True,
    controller_configs=controller_config,
    has_offscreen_renderer=False,
    use_camera_obs=False,
)

# for i in range(1000):
#     # real to sim means that we no longer use sim obs to make an action but real obs
#     action, _states = model.predict(obs, deterministic=True)
#     obs, reward, done, info = wrapped_env.step(action)  # take action in the environment
#     # need to make a ros publisher here
#     print(action)
#     wrapped_env.render()  # render on display
#     if done:
#         obs = wrapped_env.reset()

def array_publisher(wrapped_env, model):
    rospy.init_node('array_publisher', anonymous=True)
    pub = rospy.Publisher('array_topic', Array, queue_size=10)

    running = True
    count = 0
    obs = wrapped_env.reset()

    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown() and running:
        # Example array data (floats)
        array_data = np.array([1.0, 2.0, 3.0, 4.0, 5.0])

        action, _states = model.predict(obs, deterministic=True)
        obs, reward, done, info = wrapped_env.step(action)

        array_data = np.array(action)
        if done:
            obs = wrapped_env.reset()
        count += 1
        if count == 50:
            running = False

        # Create a message object
        msg = Array()
        msg.data = array_data.tolist()  # Convert numpy array to Python list

        # Publish the message
        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    wrapped_env = GymWrapper(env)

    model = SAC("MlpPolicy", wrapped_env, verbose=1)
    model.learn(total_timesteps=100, log_interval=4)

    try:
        array_publisher(wrapped_env, model)
    except rospy.ROSInterruptException:
        pass
