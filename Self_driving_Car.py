# Add libraries

import cv2
import numpy as np
import avisengine
import config
import time

# Create an instance of the Car class

car = avisengine.Car()

# Connect to the server (Simulator)

car.connect(config.SIMULATOR_IP, config.SIMULATOR_PORT)


# PID control variables

Kp = 1
Ki = 0.0
Kd = 0.9
prev_error = 0
integral = 0

def pid_control(error):
    global prev_error, integral
    integral += error  # Accumulate the integral of the error
    derivative = error - prev_error  # Calculate the derivative of the error
    prev_error = error  # Update previous error
    control = Kp * error + Ki * integral + Kd * derivative  # PID formula
    return control  # Return the control value



# PID control variables for speed

Kp_speed = 10
Ki_speed = 1
Kd_speed = 10
prev_error_speed = 0
integral_speed = 0

def pid_control_speed(error):
    global prev_error_speed, integral_speed
    integral_speed += error
    derivative = error - prev_error_speed
    prev_error_speed = error
    control = Kp_speed * error + Ki_speed * integral_speed + Kd_speed * derivative
    return int(control)  


# Line detection function

def Line_Detection(img):
    height, width = img.shape[:2]
    mask = np.zeros_like(img)  # Create a black mask with the same size as the image
    polygon = np.array([
        (int(width * 0.1), int(height * 0.9)),  # Bottom-left
        (int(width * 0.4), int(height * 0.55)),  # Top-left
        (int(width * 0.6), int(height * 0.55)),  # Top-right
        (int(width * 0.9), int(height * 0.9))  # Bottom-right
    ], np.int32)
    
    cv2.fillPoly(mask, [polygon], (255, 255, 255))  # Fill the polygon with white
    masked_image = cv2.bitwise_and(img, mask)  # Apply the mask to the image

    # Convert to grayscale
    gray = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)  # Apply Gaussian Blur to reduce noise

    # Edge detection
    edges = cv2.Canny(blur, 50, 150)

    # Hough Transform
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=40, maxLineGap=100)

    left_line = None
    right_line = None
    left_x_intercept = width
    right_x_intercept = 0

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1)  # Calculate the slope of the line
            x_intercept = (x1 + x2) // 2  # Calculate the x-intercept of the line
            if slope < 0 and x_intercept < width // 2:
                if x_intercept < left_x_intercept:
                    left_x_intercept = x_intercept
                    left_line = (x1, y1, x2, y2)
            elif slope > 0 and x_intercept > width // 2:
                if x_intercept > right_x_intercept:
                    right_x_intercept = x_intercept
                    right_line = (x1, y1, x2, y2)

    if left_line is not None:
        x1, y1, x2, y2 = left_line
        cv2.line(img, (x1, y1), (x2, y2), (255, 255, 0), 2)  # Draw the left line
        left_center = ((x1 + x2) // 2, (y1 + y2) // 2)

    if right_line is not None:
        x1, y1, x2, y2 = right_line
        cv2.line(img, (x1, y1), (x2, y2), (255, 255, 0), 2)  # Draw the right line
        right_center = ((x1 + x2) // 2, (y1 + y2) // 2)

    car_center_x = width // 2  # Calculate the center of the car

    if left_line is not None and right_line is not None:
        if car_center_x <= left_center[0]:
            right_offset_x = right_center[0] - (width // 4)
            center_point = (right_offset_x, right_center[1])
        elif car_center_x >= right_center[0]:
            left_offset_x = left_center[0] + (width // 4)
            center_point = (left_offset_x, left_center[1])
        else:
            avg_center_x = (left_center[0] + right_center[0]) // 2
            avg_center_y = (left_center[1] + right_center[1]) // 2
            center_point = (avg_center_x, avg_center_y)

        # Add to buffer
        center_points.append(center_point)

        # Keep buffer size limited
        if len(center_points) > frames_to_average:
            center_points.pop(0)

        # Compute the overall average center point
        final_center_x = sum([pt[0] for pt in center_points]) // len(center_points)
        final_center_y = sum([pt[1] for pt in center_points]) // len(center_points)

        # Draw the center point
        cv2.circle(img, (final_center_x, final_center_y), 5, (255, 255, 0), -1)

        # PID control to determine the steering angle
        error = final_center_x - car_center_x
        steering_angle = pid_control(error)
        steering_angle = int(np.clip(steering_angle, -100, 100))  # Clipping the angle to be within -100 to 100 and converting to int

        print(f'Steering Angle: {steering_angle}')
        car.setSteering(steering_angle)  # Set the steering angle of the car

        # PID control to determine the speed
        error_speed = abs(final_center_x - car_center_x)
        speed = pid_control_speed(error_speed)
        speed = int(np.clip(100 - speed, 50, 100))  # Ensure speed is between 50 and 100

        print(f'Speed: {speed}')
        car.setSpeed(speed)  # Set the speed of the car

# Initialization of center_points as empty list and frames_to_average
center_points = []
frames_to_average = 5

while (True):
    car.getData()
    frame = car.getImage()  # Capture frame by frame
    
    Line_Detection(frame)  # Detect lines
    
    cv2.imshow('Road Lines', frame)  # Display the frame with lines

    # Exit when 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources and close windows
cv2.destroyAllWindows()
