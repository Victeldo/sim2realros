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
    env_name="Lift", # try with other tasks like "Stack" and "Door"
    reward_shaping=True,
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

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion.

    Args:
    - roll: Rotation around x-axis in radians (float)
    - pitch: Rotation around y-axis in radians (float)
    - yaw: Rotation around z-axis in radians (float)

    Returns:
    - Quaternion [w, x, y, z] (list of floats)
    """

    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return [qw, qx, qy, qz]

robot_obs = [0] * 40
robot_obs = np.array(robot_obs)

def obs_callback(msg):
    global robot_obs
    robot_obs = np.array(msg.data)

def format_obs(modded_observations):

    formatted_obs = np.array([])
    count = 0
    for key in modded_observations:
        if count < 7:
            print(key)
            formatted_obs = np.concatenate((formatted_obs, modded_observations[key]))
        count += 1
    formatted_obs = np.concatenate((modded_observations["object-state"], formatted_obs))

    return formatted_obs

def array_publisher(wrapped_env, model):
    rospy.init_node('array_publisher', anonymous=True)
    pub = rospy.Publisher('array_topic', Array, queue_size=10)
    rospy.Subscriber('/my_gen3/robot_deets_topic', Array, obs_callback)
    # rospy.Subscriber('array_topic', Array, array_callback)

    # pub2 = rospy.Publisher('start_topic', Array)
    # print("yes")
    rospy.loginfo("YES")

    running = True
    count = 0
    obs = wrapped_env.reset()

    rate = rospy.Rate(1)  # 1Hz
    obs_unwrapped = wrapped_env.unwrapped
    obs_unwrapped = obs_unwrapped.reset()
    
    while not rospy.is_shutdown() and running:
    # while not rospy.is_shutdown():
    # while running:
        # Example array data (floats)
        # robot_deets = rospy.wait_for_message("robot_deets_topic", Array)
        array_data = np.array([1.0, 2.0, 3.0, 4.0, 5.0])

        obs_unwrapped = GymWrapper.observation_spec(wrapped_env)
        print(len(robot_obs))

        # getting these values made up soon for object state

        robot_obs_half = np.concatenate((obs_unwrapped["object-state"], robot_obs))

        # action, _states = model.predict(obs, deterministic=True)
        action, _states = model.predict(robot_obs_half, deterministic=True)
        obs, reward, done, info = wrapped_env.step(action)

        array_data = np.array(action)
        if done:
            obs = wrapped_env.reset()
        
        count += 1
        rospy.loginfo(count)
        if count == 50000:
            running = False

        # Create a message object
        msg = Array()
        msg.data = array_data.tolist()  # Convert numpy array to Python list

        # Publish the message
        pub.publish(msg)
        
        rospy.sleep(1.0)
        # rate.sleep()

if __name__ == '__main__':
    wrapped_env = GymWrapper(env)

    model = SAC("MlpPolicy", wrapped_env, verbose=1)
    model.learn(total_timesteps=500, log_interval=4)

    try:
        # rospy.loginfo("GO")
        array_publisher(wrapped_env, model)
        # print("WHAT")
    except rospy.ROSInterruptException:
        pass
    # print("DONE")
