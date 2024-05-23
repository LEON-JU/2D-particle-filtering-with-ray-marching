import rosbag
import rospy
import numpy as np
import yaml
import random
import math
import sys
from math import atan2, sqrt, cos, sin
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import cv2
from cv_bridge import CvBridge
from scipy.stats import norm, expon

import imageio

M_PI = math.pi

class Particle:
    def __init__(self, x, y, theta, weight):
        self.x = x
        self.y = y
        self.theta = theta
        self.w = weight

def image_to_world(coordinate):
    x = coordinate[0] * 0.05 - (300 * 0.05) / 2
    y = coordinate[1] * 0.05 - 2 - (300 * 0.05) / 2
    return(x, y)

def world_to_image(coordinate):
    x = int((coordinate[0] + (300 * 0.05) / 2) / 0.05)
    y = int((coordinate[1] + 2 + (300 * 0.05) / 2) / 0.05)
    return (x, y)

def get_odom(msg):
    robot_x = msg.pose.pose.position.x
    robot_y = msg.pose.pose.position.y

    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    robot_theta = yaw

    return (robot_x, robot_y, robot_theta)

def draw_robot(map_image, new_particles, scan_msgs):
    max_weight_particle = max(new_particles, key=lambda particle: particle.w)
    (robot_x, robot_y, robot_theta) = (max_weight_particle.x, max_weight_particle.y, max_weight_particle.theta)

    current_map = np.copy(map_image)
    current_map = draw_rays(current_map, scan_msgs, max_weight_particle)
    # transform robot from world frame to image frame
    (px, py) = world_to_image((robot_x, robot_y))
    print(px, py, robot_theta)

    # draw robot position
    cv2.circle(current_map, (px, py), 5, (0, 0, 255), -1)

    # draw robot orientation
    length = 10
    endX = int(px + length * np.cos(robot_theta))
    endY = int(py + length * np.sin(robot_theta))
    cv2.line(current_map, (px, py), (endX, endY), (0, 255, 0), 2)

    # draw rays
    return current_map

def draw_rays(map_image, z, robot_particle):
    start_angle = z.angle_min
    end_angle = z.angle_max
    interval_angle = z.angle_increment
    ranges = z.ranges
    range_min = z.range_min
    range_max = z.range_max
    num_rays = config['ray_num']

    print(range_max)
    ranges_array = np.array(ranges)
    ranges_array = np.minimum(ranges_array, 6)

    k = int( ((end_angle - start_angle) / num_rays) / interval_angle)

    for i in range(num_rays):
        scan_angle = start_angle + k * i * interval_angle

        x_laser = cos(scan_angle) * ranges_array[k*i]
        y_laser = sin(scan_angle) * ranges_array[k*i]

        x_robot = -x_laser + 0.2
        y_robot = y_laser

        start_point = world_to_image((robot_particle.x, robot_particle.y))
        end_point = world_to_image((robot_particle.x + x_robot, robot_particle.y + y_robot))
        cv2.line(map_image, start_point, end_point, (0, 100, 255), 1)  

    return map_image
        
def debug(odom_msgs, scan_msgs):
    for odom in odom_msgs:
        print(get_odom(odom))

    print("/odom messages:", len(odom_msgs))
    print("/scan messages:", len(scan_msgs))

def read_config(file_path):
    with open(file_path, 'r') as file:
        config_data = yaml.safe_load(file)
    return config_data

def synchronize_data(Odoms, Scans):
    synchronized_pairs = []

    for scan in Scans:
        scan_secs = scan.header.stamp.secs
        scan_nsecs = scan.header.stamp.nsecs
        
        closest_odom = None
        min_time_difference = float('inf')

        for odom in Odoms:
            odom_secs = odom.header.stamp.secs
            odom_nsecs = odom.header.stamp.nsecs
            
            if odom_secs == scan_secs:
                time_difference = abs(odom_nsecs - scan_nsecs)
                if time_difference < min_time_difference:
                    min_time_difference = time_difference
                    closest_odom = odom
        
        if closest_odom is not None:
                synchronized_pairs.append((closest_odom, scan))
    
    # print(len(synchronized_pairs))
    
    return synchronized_pairs

def sample_normal_distribution(sigma):
    return random.gauss(0.0, sigma)

def initialize_particles(map_image, config):
    gray_map_image = cv2.cvtColor(map_image, cv2.COLOR_BGR2GRAY)

    particle_num = config['particle_num']
    particles = []

    white_pixels = []
    height, width = gray_map_image.shape

    for y in range(height):  
        for x in range(width):  
            if gray_map_image[y, x] == 255:  
                white_pixels.append((x, y)) 

    points = random.sample(white_pixels, particle_num)

    for point in points:
        (x_world, y_world) = image_to_world((point[0], point[1]))
        particles.append(Particle(x_world, y_world, 0, 1))

    return particles

def visualize_particles(particles, map_image):
    map_with_particles = map_image.copy() 
    for particle in old_particles:
        (x, y) = (particle.x, particle.y)
        (px, py) = world_to_image((x, y))

        px = max(0, min(299, px))
        py = max(0, min(299, py))

        cv2.circle(map_with_particles, (px, py), 2, (255, 0, 0), -1)

    return map_with_particles

def sample_motion_model_odometry(odom, x_old, config):
    (x_bar_new, y_bar_new, theta_bar_new) = get_odom(odom)
    (x_bar, y_bar, theta_bar) = (x_old.x, x_old.y, x_old.theta)

    delta_rot1 = atan2(y_bar_new - y_bar, x_bar_new - x_bar) - theta_bar
    delta_trans = sqrt((x_bar - x_bar_new)**2 + (y_bar - y_bar_new)**2)
    delta_rot2 = theta_bar_new - theta_bar - delta_rot1

    # 修正 delta_rot1
    if delta_rot1 > M_PI:
        delta_rot1 -= 2 * M_PI
    if delta_rot1 < -M_PI:
        delta_rot1 += 2 * M_PI

    # 修正 delta_rot2
    if delta_rot2 > M_PI:
        delta_rot2 -= 2 * M_PI
    if delta_rot2 < -M_PI:
        delta_rot2 += 2 * M_PI

    delta_rot1_hat = delta_rot1 - sample_normal_distribution(config['rot2rot_noise'] * delta_rot1 * delta_rot1 + config['trans2rot_noise'] * delta_trans * delta_trans)
    delta_trans_hat = delta_trans - sample_normal_distribution(config['trans2trans_noise'] * delta_trans * delta_trans + config['rot2trans_noise'] * (delta_rot1 * delta_rot1 + delta_rot2 * delta_rot2))
    delta_rot2_hat = delta_rot2 - sample_normal_distribution(config['rot2rot_noise'] * delta_rot2 * delta_rot2 + config['trans2rot_noise'] * delta_trans * delta_trans)

    x = x_old.x + delta_trans_hat * cos(x_old.theta + delta_rot1_hat)
    y = x_old.y + delta_trans_hat * sin(x_old.theta + delta_rot1_hat)
    theta = x_old.theta + delta_rot1_hat + delta_rot2_hat

    x = max(-7.5, min(7.5, x))
    y = max(-7.5, min(7.5, y))
    theta = theta % (2 * np.pi)

    x_new = Particle(x, y, theta, 0)

    return x_new

def ray_casting(x_new, map_image, direction, range_min, range_max):
    x = x_new.x
    y = x_new.y

    step_size = 0.1

    current_x = x
    current_y = y

    gray_map_image = cv2.cvtColor(map_image, cv2.COLOR_BGR2GRAY)
    
    (image_x, image_y) = world_to_image((x, y))
    if 0<= image_x <300 and 0 <= image_y <300:
        if gray_map_image[image_y][image_x] < 255:
            return -1
    else:
        return -1

    while True:
        (image_x, image_y) = world_to_image((current_x, current_y))

        if gray_map_image[image_y][image_x] < 255:
            dis = sqrt((image_x - x) ** 2 + (image_y - y) ** 2)
            if dis < range_min:
                dis = range_min
            if dis > range_max:
                dis = range_max
            return dis
            
        current_x += step_size * direction[0]
        current_y += step_size * direction[1]

def phit(z, z_star, range_max, sigma_hit):
    # print("z: ", z)
    if z >= 0 and z <= range_max:
        eta = 1 / (norm.cdf(range_max, loc=z_star, scale=sigma_hit) - norm.cdf(0, loc=z_star, scale=sigma_hit))
        p = eta * norm.pdf(z, loc=z_star, scale=sigma_hit)
    else:
        p = 0
    # print("p: ", p)
    return p

def pshort(z, z_star, range_max, lambda_short):
    if z >= 0 and z <= z_star:
        eta = 1 / (expon.cdf(z_star, scale=1/lambda_short) - expon.cdf(0, scale=1/lambda_short))
        p = eta * expon.pdf(z, scale=lambda_short)
    else:
        p = 0
    return p

def pmax(z, z_small, z_max):
    if z >= z_max:
        p = 1 #/ z_small
    else:
        p = 0
    return p

def prand(z, z_max):
    if 0 <= z < z_max:
        p = 1 / z_max
    else:
        p = 0
    return p

def beam_range_finder_model(z, x_new, map_image, config):
    # q = 1
    q = 0

    # scan data
    start_angle = z.angle_min
    end_angle = z.angle_max
    interval_angle = z.angle_increment
    ranges = z.ranges
    range_min = z.range_min
    range_max = z.range_max
    num_rays = config['ray_num']


    ranges_array = np.array(ranges)
    ranges_array = np.minimum(ranges_array, range_max)

    k = int( ((end_angle - start_angle) / num_rays) / interval_angle)

    for i in range(num_rays):
        scan_angle = start_angle + k * i * interval_angle
        x_laser = cos(scan_angle) * ranges_array[k*i]
        y_laser = sin(scan_angle) * ranges_array[k*i]

        x_robot = -x_laser + 0.2
        y_robot = y_laser
        distance = sqrt(x_robot **2 + y_robot ** 2)

        x_direction = x_robot / distance
        y_direction = y_robot / distance

        z_star = ray_casting(x_new, map_image, (x_direction, y_direction), range_min, range_max)
        
        if z_star == -1:
            return 1e-5

        p = (config['z_hit'] * phit(distance, z_star, range_max, config['sigma_hit']) 
            + config['z_short'] * pshort(distance, z_star, range_max, config['lambda_short'])
            + config['z_max'] * pmax(distance, range_min, range_max)
            + config['z_rand'] * prand(distance, range_max))
        

        # # 计算四项
        # p1 = config['z_hit'] * phit(distance, z_star, range_max, config['sigma_hit'])
        # p2 = config['z_short'] * pshort(distance, z_star, range_max, config['lambda_short'])
        # p3 = config['z_max'] * pmax(distance, range_min, range_max)
        # p4 = config['z_rand'] * prand(distance, range_max)

        # # 打印结果以进行调试
        # print("p1:", p1)
        # print("p2:", p2)
        # print("p3:", p3)
        # print("p4:", p4)
        # print("p: ", p, "\n")

        # q = q * p
        q += np.log(p)
    q = num_rays / np.abs(q)
    # print(q)
    return q

def Low_variance_sampler(Xt_hat):

    draw_particles = []
    
    r = random.uniform(0, 1 / len(Xt_hat))
    c = Xt_hat[0].w
    i = 0
    for m in range(len(Xt_hat)):
        U = r + m * 1 / len(Xt_hat)
        while U > c:
            i = i + 1
            c = c + Xt_hat[i].w
        draw_particles.append(Xt_hat[i])
    return draw_particles

def particle_filter_once(data_pair, old_particles, config, map_image):
    (u, z) = data_pair
    Xt = []
    Xt_hat = []

    for m in range(len(old_particles)):
        x_new = sample_motion_model_odometry(u, old_particles[m], config)
        x_new.w = beam_range_finder_model(z, x_new, map_image, config)
        Xt_hat.append(x_new)

    Xt = Low_variance_sampler(Xt_hat)

    return Xt_hat


if __name__ == "__main__":
    frames = []

    # read bag data
    bag = rosbag.Bag('data/hw2_data.bag')
    odom_msgs = []
    scan_msgs = []
    for topic, msg, t in bag.read_messages():
        if topic == '/odom':
            odom_msgs.append(msg)
        elif topic == '/scan':
            scan_msgs.append(msg)
    
    # read config.yaml
    config_file_path = 'data/config.yaml'
    config = read_config(config_file_path)
    print(config)

    # initialize ros modules
    rospy.init_node('map_publisher', anonymous=True)
    image_pub = rospy.Publisher('/image', Image, queue_size=10)

    # initialize map_image
    map_image = cv2.imread('data/gridmap.png')
    bridge = CvBridge()

    # initialize pose
    robot_x = 0
    robot_y = 0
    robot_theta = 0

    # initialize publish rate
    rate = rospy.Rate(10)

    # synchronize data pairs
    synchronized_data = synchronize_data(odom_msgs, scan_msgs)

    # initialize particles in free spaces
    old_particles = initialize_particles(map_image, config)
    
    map_with_particles = visualize_particles(old_particles, map_image)
    frames.append(map_with_particles)

    map_msg = bridge.cv2_to_imgmsg(map_with_particles, "bgr8")
    image_pub.publish(map_msg)

    count = 1

    font = cv2.FONT_HERSHEY_SIMPLEX
    position = (10, 30)
    font_scale = 0.5
    font_color = (255, 255, 255)
    line_type = 1

    for pair in synchronized_data: # pair is composed of (odom_msg, scan_msg)
        # print("current time: ", pair[0].header.stamp)
        print(f"{count} / {len(synchronized_data)}")

        try:
            new_particles = particle_filter_once(pair, old_particles, config, map_image)
        except IndexError:
            # break
            old_particles = []
            old_particles = initialize_particles(map_image, config)
            continue

        map_with_particles = visualize_particles(new_particles, map_image)
        
        current_map = draw_robot(map_with_particles, new_particles, pair[1])

        timestamp = pair[0].header.stamp.to_sec()  # Convert ROS time to seconds
        timestamp_text = f"Time: {timestamp:.2f} sec"
        cv2.putText(current_map, timestamp_text, position, font, font_scale, font_color, line_type)
        # cv2.putText(map_with_particles, timestamp_text, position, font, font_scale, font_color, line_type)
        frames.append(current_map)

        # publish current map_image
        map_msg = bridge.cv2_to_imgmsg(current_map, "bgr8")
        image_pub.publish(map_msg)

        old_particles = new_particles

        # rate.sleep()

        count += 1
    

    bag.close()

    # 使用 imageio 将 frames 中的图像合并为 GIF 动画
    imageio.mimsave('output_robot.gif', frames, fps=10)
