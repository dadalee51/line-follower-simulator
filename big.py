import numpy as np
import pygame
import os

class Motor:
    def __init__(self, max_motor_speed, wheel_radius):
        self.max_motor_speed = max_motor_speed
        self.wheel_radius = wheel_radius
    
    def set_speed(self, speed):
        self.speed = speed


class Sensor:
    def __init__(self, sensor_relative_position, robot_initial_position):
        sensor_position_rotated = rotate_vector(sensor_relative_position, robot_initial_position[2])
        
        self.x = robot_initial_position[0] + sensor_position_rotated[0] 
        self.y = robot_initial_position[1] - sensor_position_rotated[1]
        
    def update_position(self, sensor_relative_position, robot_position):
        sensor_position_rotated = rotate_vector(sensor_relative_position, robot_position[2])
        
        self.x = robot_position[0] + sensor_position_rotated[0] 
        self.y = robot_position[1] - sensor_position_rotated[1]
    
    def read_data(self, map_image):
        color = map_image.get_at((int(self.x), int(self.y)))[:-1]
        
        self.data = 0 if is_darker(color, (255/2, 255/2, 255/2)) else 1
  
  
class Robot:
    def __init__(self, initial_position, width, 
                 initial_motor_speed=500, max_motor_speed=1000, wheel_radius=0.04):
        self.sensors = []
        
        self.meters_to_pixels = 3779.52
        
        self.width = width
        
        self.x = initial_position[0]
        self.y = initial_position[1]
        self.heading = initial_position[2]
        
        self.left_motor = Motor(max_motor_speed, wheel_radius)
        self.right_motor = Motor(max_motor_speed, wheel_radius)
        
        self.left_motor.set_speed(initial_motor_speed)
        self.right_motor.set_speed(initial_motor_speed)
        
    def update_position(self, dt):
        left_wheel_linear_speed = 2*np.pi*self.left_motor.wheel_radius*self.left_motor.speed/60
        right_wheel_linear_speed = 2*np.pi*self.right_motor.wheel_radius*self.right_motor.speed/60
        
        x_speed = (left_wheel_linear_speed + right_wheel_linear_speed)*np.cos(self.heading)/2
        y_speed = (left_wheel_linear_speed + right_wheel_linear_speed)*np.sin(self.heading)/2
        heading_speed = (right_wheel_linear_speed - left_wheel_linear_speed)/self.width
        
        self.x += x_speed*dt
        self.y -= y_speed*dt
        self.heading += heading_speed*dt
        
        if (self.heading > 2*np.pi) or (self.heading < -2*np.pi):
            self.heading = 0
        
    def move_forward(self):
        self.left_motor.set_speed(self.left_motor.max_motor_speed)
        self.right_motor.set_speed(self.right_motor.max_motor_speed)
        
    def move_backward(self):
        self.left_motor.set_speed(-self.left_motor.max_motor_speed)
        self.right_motor.set_speed(-self.right_motor.max_motor_speed)
        
    def turn_left(self):
        self.left_motor.set_speed(-self.left_motor.max_motor_speed)
        self.right_motor.set_speed(self.right_motor.max_motor_speed)
        
    def turn_right(self):
        self.left_motor.set_speed(self.left_motor.max_motor_speed)
        self.right_motor.set_speed(-self.right_motor.max_motor_speed)
    
    def stop(self):
        self.left_motor.set_speed(0)
        self.right_motor.set_speed(0)
    
    def add_sensor(self, sensor_relative_position, robot_initial_position):
        self.sensors.append(Sensor(sensor_relative_position, robot_initial_position))
        
        

class Graphics:
    def __init__(self, screen_dimensions, robot_image_path, map_imape_path):
        pygame.init()
        
        self.robot_image = pygame.image.load(robot_image_path)
        self.map_image = pygame.transform.scale(pygame.image.load(map_imape_path), screen_dimensions)
    
        pygame.display.set_caption("Line Follower Simulator")
        self.map = pygame.display.set_mode(screen_dimensions)
    
        self.map.blit(self.map_image, (0, 0))
    
    def robot_positioning(self):
        running = True
        closed = False
        robot_start_heading = np.pi/2
        xy_positioned = False
        heading_positioned = False
        
        empty_box = "\u25A1"
        filled_box = "\u25A0"
        xy_marker = empty_box
        heading_marker = empty_box
        
        while running:
            
            for event in pygame.event.get():
                
                if event.type == pygame.QUIT: 
                    running = False
                    closed = True
                
                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1: 
                    robot_start_x, robot_start_y = pygame.mouse.get_pos()
                    xy_positioned = True
                    xy_marker = filled_box
                
                if event.type == pygame.MOUSEBUTTONDOWN and event.button in [4, 5]:
                    if event.button == 4:
                        direction = 1
                    elif event.button == 5:
                        direction = -1
                    robot_start_heading += direction*np.pi/6
                    robot_start_heading = robot_start_heading % (2*np.pi)
                    heading_positioned = True
                    heading_marker = filled_box
                    
                if (event.type == pygame.KEYDOWN and 
                    xy_positioned and
                    heading_positioned):
                    running = False
                    
            self.map.blit(self.map_image, (0, 0))
            
            BOX_POSITION = (10, 80)
            BOX_SIZE = (480, 150)
            if xy_positioned and heading_positioned:
                BOX_SIZE = (480, 200)
            BOX_COLOR = (0, 0, 0)
            BOX_BACKGROUND_COLOR = (255, 255, 255)
            BOX_BORDER_WIDTH = 2
            box = pygame.Rect(BOX_POSITION, BOX_SIZE)
            
            pygame.draw.rect(self.map, BOX_BACKGROUND_COLOR, box)
            
            pygame.draw.rect(self.map, BOX_COLOR, box, BOX_BORDER_WIDTH)
            
            self.show_text(text="Position the robot:",
                        position=(20, 100), fontsize=25)
            
            self.show_text(text=f"{heading_marker} Scroll the mouse wheel to rotate the robot.",
                        position=(40, 150), fontsize=20)
            
            self.show_text(text=f"{xy_marker} Left click to position the robot.",
                        position=(40, 190), fontsize=20)
            
            if xy_positioned and heading_positioned:
                self.show_text(text="Press any key to continue.",
                            position=(20, 240), fontsize=25)
                
            if not(xy_positioned):
                mouse_x, mouse_y = pygame.mouse.get_pos()
                self.draw_robot(mouse_x, mouse_y, robot_start_heading)
            else:
                self.draw_robot(robot_start_x, robot_start_y, robot_start_heading)
                
            pygame.display.update()
            
        return (robot_start_x, robot_start_y, robot_start_heading), closed
    
    def sensors_positioning(self, number_of_sensors, robot_start, closed):
        running = True
        sensors_positions = []
        sensors_relative_positions = []
        counter = 0
        
        sensor_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255),
                        (255, 255, 0), (0, 255, 255), (255, 0, 255),
                        (255, 255, 255), (128, 0, 0), (0, 128, 0),
                        (0, 0, 128)]
        
        while counter < number_of_sensors and running and not(closed):
            
            for event in pygame.event.get():
                
                if event.type == pygame.QUIT: 
                    running = False
                    closed = True
                
                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1: 
                    
                    sensor_x, sensor_y = pygame.mouse.get_pos()
                    sensors_positions.append((sensor_x, sensor_y))
                    
                    sensor_relative_x = sensor_x - robot_start[0]
                    sensor_relative_y = robot_start[1] - sensor_y
                    sensor_relative = rotate_vector((sensor_relative_x, sensor_relative_y), -robot_start[2])
                    sensors_relative_positions.append(list(sensor_relative))
                    
                    counter += 1
            
            self.map.blit(self.map_image, (0, 0))
            
            self.draw_robot(robot_start[0], robot_start[1], robot_start[2])
            
            BOX_POSITION = (10, 80)
            BOX_SIZE = (460, 150)
            BOX_COLOR = (0, 0, 0)
            BOX_BACKGROUND_COLOR = (255, 255, 255)
            BOX_BORDER_WIDTH = 2
            box = pygame.Rect(BOX_POSITION, BOX_SIZE)
            
            pygame.draw.rect(self.map, BOX_BACKGROUND_COLOR, box)
            
            pygame.draw.rect(self.map, BOX_COLOR, box, BOX_BORDER_WIDTH)
            
            self.show_text(text="Position the sensors:",
                        position=(20, 100), fontsize=25)
            
            self.show_text(text="Left click to position each sensor.",
                        position=(40, 150), fontsize=20)
            
            self.show_text(text=f"Positioned: {counter}/{number_of_sensors}",
                        position=(20, 190), fontsize=25)
            
            for sensor in sensors_positions:
                self.draw_sensor_symbol((sensor[0], sensor[1]), color=sensor_colors[sensors_positions.index(sensor)])
                    
            mouse_x, mouse_y = pygame.mouse.get_pos()
            if counter < number_of_sensors:
                color_counter = counter
            else:
                color_counter = number_of_sensors - 1
            self.draw_sensor_symbol((mouse_x, mouse_y), color=sensor_colors[color_counter])
            
            pygame.display.update()
            
        return sensors_relative_positions, closed
    
    def draw_robot(self, x, y, heading):
        rotated_robot = pygame.transform.rotozoom(self.robot_image, np.degrees(heading), 1)
        
        rect = rotated_robot.get_rect(center=(x, y))
        
        self.map.blit(rotated_robot, rect)
        
    def draw_sensor(self, sensor, color=(255, 0, 0)):
        position = (int(sensor.x), int(sensor.y))
        self.draw_sensor_symbol(position, color)
        
    def draw_sensor_symbol(self, position, color=(255, 0, 0)):
        pygame.draw.circle(self.map, (0, 0, 0), (position[0], position[1]), 6)
        pygame.draw.circle(self.map, color, (position[0], position[1]), 5)
        
    def show_sensors_data(self, sensors, sensor_colors):
        font = pygame.font.SysFont("Arial", 20)
        
        text = []
        text_counter = 0
        for sensor in sensors:
            text.append(font.render(f"{text_counter}  = "+ str(sensor.data), True, (0, 0, 0)))
            text_counter += 1
        
        BOX_POSITION = (10, 35)
        BOX_SIZE = (100, 30 + 20*len(text))
        BOX_COLOR = (0, 0, 0)
        BOX_BACKGROUND_COLOR = (255, 255, 255)
        BOX_BORDER_WIDTH = 2
        box = pygame.Rect(BOX_POSITION, BOX_SIZE)
        
        pygame.draw.rect(self.map, BOX_BACKGROUND_COLOR, box)
        
        pygame.draw.rect(self.map, BOX_COLOR, box, BOX_BORDER_WIDTH)
        
        text_number = len(text)
        for idx in range(text_number):
            self.draw_sensor_symbol((30, 58 + 20*idx), color=sensor_colors[idx])
            self.map.blit(text[idx], (40, 50 + 20*idx))
            
    def is_out_of_bounds(self, object):
        if (object.x < 0 or
            object.x > self.map.get_width() or
            object.y < 0 or
            object.y > self.map.get_height()):
            return True
        else:
            return False
        
    def show_important_message(self, message):
        font = pygame.font.SysFont("Arial", 30)
        text = font.render(message, True, (0, 0, 0))

        text_rect = text.get_rect(center=(self.map.get_width()/2, self.map.get_height()/2))

        border_rect = pygame.Rect(text_rect.left - 15, text_rect.top - 15, text_rect.width + 30, text_rect.height + 30)
        pygame.draw.rect(self.map, (0, 0, 0), border_rect)

        pygame.draw.rect(self.map, (255, 255, 255), (text_rect.left - 12, text_rect.top - 12, text_rect.width + 24, text_rect.height + 24))

        self.map.blit(text, text_rect)
        
    def show_text(self, text, position, fontsize=30, color=(0, 0, 0)):
        font = pygame.font.SysFont("Arial", fontsize)
        text = font.render(text, True, color)
        self.map.blit(text, position)

def PID(kp, ki, kd, I,
        error, last_error, dt):
    if dt == 0:
        dt+=1e-5
    
    P = kp*error
    D = kd*(error - last_error)/dt
    I += ki*error*dt
 
    return P + D + I, I

def rotate_vector(vector, angle):
    rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                [np.sin(angle), np.cos(angle)]])
    
    rotated_vector = rotation_matrix.dot(vector)
    
    return rotated_vector

def is_darker(color1, color2):
    gray1 = sum(color1) / len(color1)
    gray2 = sum(color2) / len(color2)
    
    return gray1 < gray2

def main():
    # Define all the values here instead of reading from setup.txt
    ROBOT_WIDTH = 0.1
    INITIAL_MOTOR_SPEED = 10000
    MAX_MOTOR_SPEED = 20000
    WHEEL_RADIUS = 0.04
    SENSORS_NUMBER = 5
    SENSOR_COLORS = [(255, 0, 0), (0, 255, 0), (0, 0, 255),
                    (255, 255, 0), (0, 255, 255), (255, 0, 255),
                    (255, 255, 255), (128, 0, 0), (0, 128, 0),
                    (0, 0, 128)]

    pygame.init()
    infoObject = pygame.display.Info()
    MAP_DIMENSIONS = (infoObject.current_w - 30, infoObject.current_h - 100)
    gfx = Graphics(MAP_DIMENSIONS, 'images/robot.png', 'images/map.png')

    ROBOT_START, closed = gfx.robot_positioning()

    SENSORS_POSITIONS, closed = gfx.sensors_positioning(SENSORS_NUMBER, ROBOT_START, closed)

    if closed:
        print("\033[91m {}\033[00m" .format("\nThe setup was not completed! Exiting...\n"))
        return

    robot = Robot(initial_position=ROBOT_START,
                  width=ROBOT_WIDTH,
                  initial_motor_speed=INITIAL_MOTOR_SPEED,
                  max_motor_speed=MAX_MOTOR_SPEED,
                  wheel_radius=WHEEL_RADIUS)

    for position in SENSORS_POSITIONS:
        robot.add_sensor(position, ROBOT_START)

    last_time = pygame.time.get_ticks()
    last_error = 0
    I = 0

    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        gfx.map.blit(gfx.map_image, (0, 0))
        
        gfx.draw_robot(robot.x, robot.y, robot.heading)
        
        for sensor in robot.sensors:
            gfx.draw_sensor(sensor, color=SENSOR_COLORS[robot.sensors.index(sensor)])
        
        for idx in range(len(robot.sensors)):
            robot.sensors[idx].read_data(gfx.map_image)
        
        gfx.show_sensors_data(robot.sensors, sensor_colors=SENSOR_COLORS[:SENSORS_NUMBER])

        current_time = pygame.time.get_ticks()
        dt = (current_time - last_time)/1000
        last_time = current_time

        error = robot.sensors[1].data - robot.sensors[3].data
        
        pid, I = PID(kp=50, ki=3, kd=0.01, I=I,
                    error=error, last_error=last_error, dt=dt)
        
        last_error = error
        
        robot.left_motor.set_speed(robot.left_motor.max_motor_speed + pid)
        robot.right_motor.set_speed(robot.right_motor.max_motor_speed - pid)
        
        robot.update_position(dt)
        
        for idx in range(len(robot.sensors)):
            robot.sensors[idx].update_position(robot_position=(robot.x, robot.y, robot.heading),
                            sensor_relative_position=SENSORS_POSITIONS[idx])
        
        robot_is_out = gfx.is_out_of_bounds(robot)
        sensor_is_out = bool(np.sum([gfx.is_out_of_bounds(sensor) for sensor in robot.sensors]))
        
        if robot_is_out or sensor_is_out:
            gfx.show_important_message("The robot went off the map!")
            
            pygame.display.update()
            pygame.time.wait(3500)
            running = False
            
        if running:
            pygame.display.update()

    pygame.quit()

if __name__ == "__main__":
    main()