# Team: Michelle Zheng, Maya Borowicz, Faith Mulugeta, Yurie Han
# Class: COMP/ELEC 424
# Final Project
# Fall 2023
#
#
# Code drawn from:
# https://www.hackster.io/colonel-hackers/autonomous-rc-car-elec-424-final-project-732fd1
# https://www.hackster.io/paul-walker/autonomous-rc-car-eebfb4?f=1
# https://www.hackster.io/really-bad-idea/autonomous-path-following-car-6c4992

# Import Packages
import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
import sys
import time
import os
import board
import busio
import adafruit_mcp4728
import RPi.GPIO as GPIO

# Setup DAC
i2c = busio.I2C(board.SCL, board.SDA)
mcp4728 = adafruit_mcp4728.MCP4728(i2c, 0x64)


# Manage car behaviour
speed_encode = True

# Encoder
encoder_path = "/sys/module/gpiod_driver_encoder/parameters/encoder_val"
encoder_target_rotation = 4046040
encoder_rotation_variance = 1500000 
encoder_max_rotation = encoder_target_rotation + encoder_rotation_variance

# Camera
frame_width = 160
frame_height = 120
cam_idx = 0

# PD variables
kp = 0.09
kd = kp * 0.5

# Speed Values
zero_speed = int(65535 / 2) # car is stopped
base_speed = int(65535 / 1.87) # car moves forward slowly
speed_variance = 100
zero_turn = int(65535 / 2) # neutral steering

# Max number of loop
max_ticks = 4000

class NotSudo(Exception):
    pass

def set_speed(speed):
    # Set the speed of the car
    mcp4728.channel_b.value = speed 

def set_turn(turn):
    # Turn wheels
    mcp4728.channel_a.value = turn 

def reset_car():
    # set speed to zero and steering to neutral
    set_speed(zero_speed)
    set_turn(zero_turn)
    print("Car: stopped & straightened")    

def start_car():
    # Set speed to base speed and steering to neutral
    print("Car ready")
    set_turn(zero_turn)
    set_speed(base_speed)

def manage_speed():
    # Adjust the speed based on the speed encoder
    # read the file with encoder data
    f = open(encoder_path, "r")
    enc = int(f.readline())
    f.close()
    ret = enc

    # If within bounds (wait for car to start / stop)
    ret = base_speed
    if enc <= encoder_max_rotation:

        # speed encode based on percentage
        enc_pct_rotation = (enc - (encoder_target_rotation - encoder_rotation_variance)) / (2 * encoder_rotation_variance)
        ret = int(((speed_variance * 2) * abs(enc_pct_rotation - 1)) + (base_speed - speed_variance))

        # If speed from encoder is too fast, use base speed
        if ret > base_speed:
            ret = base_speed

    return ret

def wait(wait_time):
    # Wait for a period of time
    start_time = time.perf_counter() # start
    end_time = start_time + wait_time # end
    # loop until finished
    while (time.perf_counter() < end_time):
        pass
    return
    
def detect_edges(frame):
    # Detetct the blue lines
    # filter for blue lane lines
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # This color values were adjusted from Paul Walker Team code
    lower_blue = np.array([90, 50, 50], dtype="uint8")
    upper_blue = np.array([130, 255, 255], dtype="uint8")

    # make mask
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Get edges
    edges = cv2.Canny(mask, 50, 100)
    return edges

def region_of_interest(edges):
    # Find region to look at
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # only focus lower half of the screen
    polygon = np.array([[
        (0, height),
        (0, height / 2),
        (width, height / 2),
        (width, height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    
    # Get region to look at 
    cropped_edges = cv2.bitwise_and(edges, mask)

    return cropped_edges

def detect_line_segments(cropped_edges):
    # Find line segments

    # set constants
    rho = 1
    theta = np.pi / 180
    min_threshold = 10
    
    #get lines
    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold,
                                    np.array([]), minLineLength=5, maxLineGap=150)

    return line_segments

def average_slope_intercept(frame, line_segments):
    # finding slope of the line
    lane_lines = []

    if line_segments is None:
        print("No line segments detected")

        return lane_lines

    # set boundries
    height, width, _ = frame.shape
    left_fit = []
    right_fit = []
    boundary = 1 / 3
    left_region_boundary = width * (1 - boundary)
    right_region_boundary = width * boundary

    # go through line segments and get line of best fit
    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                continue

            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)

            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))
    
    # take average line of best fit
    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    return lane_lines

def make_points(frame, line):
    # For getting visual of lines
    height, width, _ = frame.shape

    slope, intercept = line

    y1 = height  # bottom of the frame
    y2 = int(y1 / 2)  # make points from middle of the frame down

    if slope == 0:
        slope = 0.1

    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)

    return [[x1, y1, x2, y2]]

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=6):
    # Get visual of lines
    line_image = np.zeros_like(frame)

    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)

    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)

    return line_image

def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5):
    # Show a hedaing line
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    steering_angle_radian = steering_angle / 180.0 * math.pi

    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image

def get_steering_angle(frame, lane_lines):
    # Get the angle to steer towards
    height, width, _ = frame.shape

    if len(lane_lines) == 2:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)

    elif len(lane_lines) == 1:
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
        y_offset = int(height / 2)

    elif len(lane_lines) == 0:
        x_offset = 0
        y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
    steering_angle = angle_to_mid_deg + 90

    return steering_angle

def plot_pd(p_vals, d_vals, error, show_img=False):
    # Plot the proportional, derivative and error
    fig, ax1 = plt.subplots()
    t_ax = np.arange(len(p_vals))
    ax1.plot(t_ax, p_vals, '-', label="P values")
    ax1.plot(t_ax, d_vals, '-', label="D values")
    ax2 = ax1.twinx()
    ax2.plot(t_ax, error, '--r', label="Error")

    ax1.set_xlabel("Frames")
    ax1.set_ylabel("PD Value")
    ax2.set_ylim(-90, 90)
    ax2.set_ylabel("Error Value")

    plt.title("PD Values over time")
    fig.legend()
    fig.tight_layout()
    plt.savefig("pd_plot.png")

    if show_img:
        plt.show()
    plt.clf()

def plot_pwm(speed_pwms, turn_pwms, error, show_img=False):
    # Plot teh speed steering and the error
    fig, ax1 = plt.subplots()
    t_ax = np.arange(len(speed_pwms))
    ax1.plot(t_ax, speed_pwms, '-', label="Speed Voltage")
    ax1.plot(t_ax, turn_pwms, '-', label="Steering Voltage")
    ax2 = ax1.twinx()
    ax2.plot(t_ax, error, '--r', label="Error")

    ax1.set_xlabel("Frames")
    ax1.set_ylabel("Voltage")
    ax2.set_ylabel("Error Value")

    plt.title("Voltage over time")
    fig.legend()
    plt.savefig("voltage_plot.png")

    if show_img:
        plt.show()
    plt.clf()

# adapted from hackster, changed to adhere to red HSV color segments
def isRedFloorVisible(image):
    """
    Detects whether or not the majority of a color on the screen is a particular color
    :param image:
    :param boundaries: [[color boundaries], [success boundaries]]
    :return: boolean if image satisfies provided boundaries, and an image used for debugging
    """
    # Convert to HSV color space
    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    cv2.imwrite("redfloor.jpg", hsv_img)

    # parse out the color boundaries and the success boundaries
    # percentage was adjusted
    percentage = 25

    # lower and upper range for the lower part of red
    lower_red1 = np.array([0, 40, 60], dtype="uint8")
    upper_red1 = np.array([10, 255, 255], dtype="uint8")

    # lower and upper range for the upper part of red
    lower_red2 = np.array([170, 40, 60], dtype="uint8")
    upper_red2 = np.array([180, 255, 255], dtype="uint8")

    # create two masks to capture both ranges of red
    mask1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv_img, lower_red2, upper_red2)

    # combining the masks
    mask = cv2.bitwise_or(mask1, mask2)

    # applying the mask
    output = cv2.bitwise_and(hsv_img, hsv_img, mask=mask)

    # save the output image
    cv2.imwrite("redfloormask.jpg", output)

    # calculating what percentage of image falls between color boundaries
    percentage_detected = np.count_nonzero(mask) * 100 / np.size(mask)
    # if the percentage percentage_detected is betweeen the success boundaries, we return true,
    # otherwise false for result
    result = percentage < percentage_detected
    if result:
        print(percentage_detected)
    return result, output


def main():
    #setup variables
    lastTime = 0
    lastError = 0
    SecondStopTick = 0


    # arrays for making the final graphs
    p_vals = [] # proportional
    d_vals = [] # Derivative
    err_vals = [] # error
    speed_vals = [] # speed values
    steer_vals = [] # steering values


    # set up video
    video = cv2.VideoCapture(cam_idx)
    video.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    video.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

    #Starts car
    start_car()
    curr_speed = base_speed

    counter = 0
    passedFirstStopSign = False
    while counter < max_ticks:
           set_speed(zero_speed)
           # manage video
           ret, original_frame = video.read()
           frame = cv2.resize(original_frame, (160, 120))

           # process the frame to determine the desired steering angle
           edges = detect_edges(frame)
           cv2.imshow("edges", edges)
           roi = region_of_interest(edges)
           line_segments = detect_line_segments(roi)
           lane_lines = average_slope_intercept(frame, line_segments)
           lane_lines_image = display_lines(frame, lane_lines)

           steering_angle = get_steering_angle(frame, lane_lines)
           heading_image = display_heading_line(lane_lines_image,steering_angle)

           # calculate changes for PD
           now = time.time()
           dt = now - lastTime
           deviation = steering_angle - 90

           # PD Code
           error = -deviation
           base_turn = 7.5
           proportional = kp * error
           derivative = kd * (error - lastError) / dt

           # Get Speed voltage
           speed_voltage = curr_speed*(3.3/65535)


           # take values for graphs
           p_vals.append(proportional)
           d_vals.append(derivative)
           err_vals.append(error)
           speed_vals.append(speed_voltage)


           # determine actual turn to do
           turn_amt = base_turn + proportional + derivative
        
           # makes trun based on turn_amt
           # Inspiration for sigmoid from Team Colonel Hackers
           turn_amt = int(1/(1 + np.exp(0.32*(turn_amt -9.4)))*65535)
           steer_voltage = turn_amt* (3.3/65535)
           steer_vals.append(steer_voltage)
           set_turn(turn_amt)
           
           # Stops if sees the red box
           if counter % 20: # looks every 20 ticks
                if not passedFirstStopSign:
                    isStopSign, floorSight = isRedFloorVisible(frame)

                    # Sees red box
                    if isStopSign:
                        wait(3)
                        passedFirstStopSign = True
                        SecondStopTick = counter
                
                # for last box       
                elif passedFirstStopSign and counter > SecondStopTick+100:
                    isStopSign, _ = isRedFloorVisible(frame)
                    if isStopSign:
                        set_speed(zero_speed)
                        break # Stop forever

           # speed encoding
           if speed_encode:
                if counter % 3 == 0: # Check every 3 ticks

                    # adjust speed
                    temp_speed = manage_speed()
                    if temp_speed != curr_speed:
                        set_speed(temp_speed)
                        curr_speed = temp_speed

           set_speed(curr_speed)
           wait(0.023) # Run at speed for a short amount of time

           key = cv2.waitKey(1)
           if key == 27:
                break

           counter += 1

    # Reset car and close everything
    reset_car()
    video.release()
    cv2.destroyAllWindows()

    # save plots
    plot_pd(p_vals, d_vals, err_vals)
    plot_pwm(speed_vals, steer_vals, err_vals)
 
if __name__ == "__main__":
    main()