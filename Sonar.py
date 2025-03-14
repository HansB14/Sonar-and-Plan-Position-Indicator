import pygame
import numpy as np
import time
from pymata4 import pymata4

# Initialize Arduino board
board = pymata4.Pymata4()

# Setup HC-SR04 sensor (Trig: 11, Echo: 12)
TRIG_PIN = 11
ECHO_PIN = 12
sonar_distance = 0  # Variable to store sensor data
MAX_DISTANCE = 100  # 100 cm should be the edge of the radar

# Setup Servo (Signal Pin: 9)
SERVO_PIN = 9
board.set_pin_mode_servo(SERVO_PIN)

# Function to update sonar distance
def sonar_callback(data):
    global sonar_distance
    sonar_distance = data[2]  # Extract distance value

board.set_pin_mode_sonar(TRIG_PIN, ECHO_PIN, sonar_callback)

# Pygame Setup
WIDTH, HEIGHT = 600, 600
CENTER = (WIDTH // 2, HEIGHT // 2)
RADIUS = 250  # Radar range on screen
ANGLE_STEP = 5  # Rotation step in degrees
FADE_DURATION = 2  # Time in seconds for dots to fully fade

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()
font = pygame.font.Font(None, 24)

def scale_distance(distance):
    """Linearly scale sonar distance to fit the radar screen"""
    return min((distance / MAX_DISTANCE) * RADIUS, RADIUS)  # Ensure within bounds


def polar_to_cartesian(angle, distance):
    """Convert polar coordinates to Cartesian for plotting
    Adjust angle so 90° is straight right (pointing right on the screen).
    """
    angle_rad = np.radians(angle - 90)  # Adjust angle by -90° to rotate radar start to right (90°)
    scaled_distance = scale_distance(distance)  # Apply linear scaling
    x = CENTER[0] + scaled_distance * np.cos(angle_rad)
    y = CENTER[1] - scaled_distance * np.sin(angle_rad)  # Invert Y-axis for screen
    return int(x), int(y)


# List to store (angle, distance, timestamp) for sonar readings
sonar_data = []

# Rotation Variables for Servo Simulation
angle = 0  # Start at 0 degrees (pointing right)
direction = 1  # 1 for clockwise, -1 for counterclockwise

# Angle Offset for Servo Alignment
ANGLE_OFFSET = 0  # Adjust this value based on your servo's misalignment
SERVO_INITIAL_ANGLE = 0  # Servo starts at 0° (pointing right)

# Send the servo to 0 degrees at the start to align it properly
board.servo_write(SERVO_PIN, SERVO_INITIAL_ANGLE)

running = True

while running:
    screen.fill((0, 0, 0))  # Clear screen
    pygame.draw.circle(screen, (0, 255, 0), CENTER, RADIUS, 1)  # Radar border
    pygame.draw.line(screen, (0, 255, 0), CENTER, polar_to_cartesian(angle, RADIUS), 2)  # Radar beam

    # Get sonar reading (allow time for callback update)
    time.sleep(0.05)
    timestamp = time.time()  # Get current time
    sonar_data.append((angle, sonar_distance, timestamp))  # Store new data

    # Draw sonar points with fading effect
    current_time = time.time()
    new_sonar_data = []
    for ang, dist, ts in sonar_data:
        if 0 < dist <= MAX_DISTANCE:  # Ignore out-of-range values
            fade_factor = max(0, 1 - (current_time - ts) / FADE_DURATION)  # Calculate fade (0 to 1)
            color_intensity = int(255 * fade_factor)  # Scale brightness
            x, y = polar_to_cartesian(ang, dist)
            pygame.draw.circle(screen, (0, color_intensity, 0), (x, y), 3)  # Draw faded dot
        if current_time - ts < FADE_DURATION:  # Keep data only if it's not too old
            new_sonar_data.append((ang, dist, ts))

    sonar_data = new_sonar_data  # Update data list with only recent points

    # Draw text
    text = font.render(f"Angle: {angle}°  Distance: {sonar_distance} cm", True, (0, 255, 0))
    screen.blit(text, (10, 10))

    pygame.display.flip()  # Update display

    # Update rotation angle with alternating direction
    angle += ANGLE_STEP * direction

    # Check angle limits: Rotate between 0° and 180° then back to 0°
    if angle >= 270 or angle <= 90:  # Limits for counterclockwise and clockwise motion
        direction *= -1  # Reverse direction at the limits
        angle = max(90, min(angle, 270))  # Keep angle within 90 to 270 degrees

    # Apply the offset to the servo control (for alignment)
    servo_angle = angle + ANGLE_OFFSET  # Add the offset to align the servo with the radar


    # Sync servo with radar angle (Map angle to servo range with offset)
    board.servo_write(SERVO_PIN, servo_angle - 90)  # Control servo position

    clock.tick(10)  # Control FPS

    # Check for exit
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

pygame.quit()
board.shutdown()
