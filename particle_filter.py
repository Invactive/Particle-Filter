import pygame
import random
from math import *

display_width = 800
display_height = 800
world_size = display_width

red = (200, 0, 0)
blue = (0, 0, 255)
green = (0, 155, 0)
yellow = (255, 255, 0)
white = (255, 255, 255)
black = (0, 0, 0)

robot_length = 60
robot_width = 60

robot_img = pygame.image.load("santa.jpg")
robot_img = pygame.transform.scale(robot_img, (robot_width, robot_length))

origin = (display_width / 2, display_height / 2)

pygame.init()

screen = pygame.display.set_mode((display_width, display_height))
pygame.display.set_caption("Moving robot")
clock = pygame.time.Clock()

screen.fill(white)


class robot:
    def __init__(self):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi

        self.forward_noise = 0.0
        self.turn_noise = 0.0
        self.sense_noise = 0.0  # or bearing_noise

    def set(self, x, y, orientation):
        if x >= world_size or x < 0:
            raise ValueError('X coordinate out of bound')
        if y >= world_size or y < 0:
            raise ValueError('Y coordinate out of bound')
        if orientation >= 2 * pi or orientation < 0:
            raise ValueError('Orientation must be in [0..2pi]')

        self.x = x
        self.y = y
        self.orientation = orientation

    def set_noise(self, f_noise, t_noise, s_noise):
        self.forward_noise = f_noise
        self.turn_noise = t_noise
        self.sense_noise = s_noise

    def move(self, turn, forward):
        # movement and prediction of next position and orientation
        self.orientation = self.orientation + turn + random.gauss(0.0, self.turn_noise)
        self.orientation %= 2 * pi

        dist = forward + random.gauss(0.0, self.forward_noise)
        self.x = self.x + dist * cos(self.orientation)
        self.y = self.y - dist * sin(self.orientation)

        self.x %= world_size
        self.y %= world_size

        # return instance of robot class to use its methods later
        particle_instance = robot()
        particle_instance.set(self.x, self.y, self.orientation)
        particle_instance.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)

        return particle_instance

    def measurement(self):
        # measure distance from robot (santa_claus) to landmarks
        to_landmarks = []

        for i in range(len(landmarks_loc)):
            dx = self.x - landmarks_loc[i][0]
            dy = self.y - landmarks_loc[i][1]
            dist = sqrt(dx * dx + dy * dy) + random.gauss(0.0, self.sense_noise)
            to_landmarks.append(dist)

        return to_landmarks

    def calculate_weight(self, percept):
        # probability of accurate measurement (weight of particle)
        cur_w = 1.0

        for i in range(len(landmarks_loc)):
            dx = self.x - landmarks_loc[i][0]
            dy = self.y - landmarks_loc[i][1]
            dist = sqrt(dx * dx + dy * dy)
            diff = dist - percept[i]
            cur_w *= exp(-0.5 * (diff ** 2) / (self.sense_noise ** 2))

        return cur_w

def draw_robot(robot):
    x = robot.x
    y = robot.y
    orientation = robot.orientation
    img = pygame.transform.rotate(robot_img, orientation * 180 / pi)
    screen.blit(img, (x - robot_width/2, y - robot_length/2))

def draw_particles(particles):
    for i in range(len(particles)):
        x = particles[i].x
        y = particles[i].y
        orientation = particles[i].orientation
        pygame.draw.circle(screen, green, (int(x), int(y)), 5)

landmarks_loc = [[200, 200], [600, 600], [200, 600], [600, 200], [200, 300], [300, 200], [500, 200], \
                 [200, 200], [200, 500], [300, 600], [500, 600], [600, 500], [600, 300], [400, 200], \
                 [200, 400], [400, 600], [600, 400]]

# create robot
santa_claus = robot()

# amount of particles
n = 200
particles = []

# create particles and set its noises (parameters)
for i in range(n):
    p = robot()
    p.set_noise(1.7, 1.7, 30.0)
    particles.append(p)

orientation = 0
orientation = orientation * pi / 180
santa_claus.set(origin[0], origin[1], orientation)

exit = False

delta_orient = 0.0
delta_forward = 0.0

while exit == False:

    screen.fill(white)
    pygame.draw.line(screen, green, (display_width / 2, 0), (display_width / 2, display_height), 1)
    pygame.draw.line(screen, black, (0, display_height / 2), (display_width, display_height / 2), 1)
    for i in range(len(landmarks_loc)):
         pygame.draw.circle(screen, blue, landmarks_loc[i], 20)

    # draw elements
    draw_robot(santa_claus)
    draw_particles(particles)

    pygame.display.update()
    clock.tick(100)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            exit = True
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                delta_orient = 0.0175
            elif event.key == pygame.K_RIGHT:
                delta_orient = -0.0175
            elif event.key == pygame.K_UP:
                delta_forward = 1
            elif event.key == pygame.K_DOWN:
                delta_forward = -1
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_RIGHT or event.key == pygame.K_LEFT or event.key == pygame.K_UP \
                    or event.key == pygame.K_DOWN:
                delta_orient = 0.0
                delta_forward = 0.0

    #robot (santa_claus) motion
    santa_claus.move(delta_orient, delta_forward)

    # measure distance from robot (santa_claus) to landmarks (with noise)
    perc = santa_claus.measurement()

    # simulate motion of every particle and calculate weights
    p_temp = []
    weights = []
    for i in range(n):
        p = particles[i].move(delta_orient, delta_forward)
        p_temp.append(p)

        w = particles[i].calculate_weight(perc)
        weights.append(w)
    particles = p_temp

    # resampling
    p_new = []
    idx = 0
    beta = 0.0
    w_max = max(weights)
    for i in range(n):
        beta += random.random() * 2 * w_max
        while beta > weights[idx]:
            beta -= weights[idx]
            idx = (idx + 1) % n
        p_new.append(particles[idx])
    particles = p_new

    # particles mean distance to robot (santa_claus)
    sum = 0
    for i in range(n):
        dx = particles[i].x - santa_claus.x
        dy = particles[i].y - santa_claus.y
        error = sqrt(dx * dx + dy * dy)
        sum += error
    mean_error = sum / n
    print("Average error:", mean_error)
