import os
import time
import datetime
import argparse

from threading import Thread, Event

import serial
from serial.tools import list_ports

import pygame
from pygame._sdl2 import Window

import math
from tqdm import tqdm
import csv

WINDOW_SIZE = (800, 700)
WINDOW_CAPTION = "GPSM: Game Pad Stats Measurer"

DEFAULT_SERIAL_BAUD_RATE = 115200

REVERSE_MODE = True

SONY_DUALSHOCK_STICK_HEIGHT_MM = 23.0
SONY_DUALSHOCK2_STICK_HEIGHT_MM = 23.0
SONY_DS4_STICK_HEIGHT_MM = 20
SONY_DS_EDGE_STICK_HEIGHT_MM = 20
MOJHON_BLITZ2_STICK_HEIGHT_MM = 20.7
MOJHON_RAINBOW_3_STICK_HEIGHT_MM = 22.5
GAMESIR_CYLONE_2_STICK_HEIGHT_MM = 23.0
GAMESIR_G7_SE_STICK_HEIGHT_MM = 22.5
GAMESIR_G7_PRO_STICK_HEIGHT_MM = 23.5
GAMESIR_NOVA2_LITE_STICK_HEIGHT_MM = 19.5
GAMESIR_TEGENARIA_LITE_STICK_HEIGHT_MM = 22
E8BitDo_ULTIMATE2_WIRELESS_STICK_HEIGHT_MM = 19
NACON_REVOLUTION_X_UNLIMITED_STICK_HEIGHT_MM = 28.5
ZUIKI_EVOTOP_STICK_HEIGHT_MM = 21
RAZER_WOLVERINE_V3_PRO_STICK_HEIGHT_MM = 22
RAZER_WOLVERINE_V3_TE_8K_STICK_HEIGHT_MM = 22
ZD_O_PLUS_STICK_HEIGHT_MM = 20
ZD_O_PLUS_LOW_PROFILE_STICK_HEIGHT_MM = 25
ZD_ULTIMATE_LEGEND_STICK_HEIGHT_MM = 20
GULIKIT_ES_PRO_STICK_HEIGHT_MM = 22.5
FLYDIGI_APEX5_STICK_HEIGHT_MM = 21.5

# CHANGE THIS BEFORE USE
STICK_HEIGHT_MM = GAMESIR_G7_PRO_STICK_HEIGHT_MM
STICK_RADIUS_MM = 11


MEASURE_FRAME_RATE = 100

TIME_STICK_MOVEMENT_MS = 4
DEFAULT_BEFORE_SENSE_MS = 4
DEFAULT_REPEAT_TIMES = 10
DEFAULT_REPEAT_INTERVAL_MS = 1

DEFAULT_NUM_STEP = 1
STEP_DISTANCE_MM = 0.025

LINEAR_CURVE_MAX_DEGREE = 21.1596 #21.1596
LINEAR_CURVE_MAX_DISTANCE = 7.62
LINEAR_CURVE_CENTER_MAX_DISTANCE = 6.782

STICK_THRESHOLD = 0.999
STICK_END_THRESHOLD = 0.99

BUTTONS_MAP = {'A': 0, 'B': 1, 'X': 2, 'Y': 3, 'SELECT': 4, 'HOME': 5, 'START': 6, 'LS': 7, 'RS': 8, 'LB': 9, 'RB': 10, 'UP': 11, 'DOWN': 12, 'LEFT': 13, 'RIGHT': 14, 'TOUCHPAD': 15}

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-l", "--list", help="list com ports and controllers",
                    action="store_true")
    parser.add_argument("-c", "--controller", help="select controller",
                    type=int, default=0)
    parser.add_argument("-p", "--com_port", help="select com port",
                    type=int, default=0)
    parser.add_argument("-a", "--axis", help="select joystick axis",
                    default="rx")
    return parser.parse_args()

def fix_stick_val(val):
    if 0 < val:
        return (math.ceil(val * 100000) / 100000 + 0.00001) / 0.99998
    elif val < 0:
        return (math.floor(val * 100000) / 100000 + 0.00002) / 0.99998
    return val

def measure_stats(joystick, stats, cur_ms, elapsed_time, motor_pos):
    sum_lx = 0; sum_ly = 0; sum_rx = 0; sum_ry = 0
    sum_lt = 0; sum_rt = 0

    pygame.time.wait(DEFAULT_BEFORE_SENSE_MS)

    for i in range(0, DEFAULT_REPEAT_TIMES, 1):
        sum_lx += fix_stick_val(joystick.get_axis(0))
        sum_ly += fix_stick_val(joystick.get_axis(1))
        sum_rx += fix_stick_val(joystick.get_axis(2))
        sum_ry += fix_stick_val(joystick.get_axis(3))
        sum_lt += fix_stick_val(joystick.get_axis(4))
        sum_rt += fix_stick_val(joystick.get_axis(5))

        pygame.time.wait(DEFAULT_REPEAT_INTERVAL_MS)

    lx = sum_lx / DEFAULT_REPEAT_TIMES
    ly = sum_ly / DEFAULT_REPEAT_TIMES
    rx = sum_rx / DEFAULT_REPEAT_TIMES
    ry = sum_ry / DEFAULT_REPEAT_TIMES
    lt = sum_lt / DEFAULT_REPEAT_TIMES
    rt = sum_rt / DEFAULT_REPEAT_TIMES

    stats["timestamps"].append(cur_ms)
    stats["motor_pos"].append(motor_pos)
    stats["elapsed_time"].append(elapsed_time)
    stats["lx"].append(lx)
    stats["ly"].append(ly)
    stats["rx"].append(rx)
    stats["ry"].append(ry)
    stats["lt"].append(lt)
    stats["rt"].append(rt)

    return {
        "timestamps": cur_ms,
        "elapsed_time": elapsed_time,
        "lx": lx, "ly": ly,
        "rx": rx, "ry": ry,
        "lt": lt, "rt": rt
    }


def stick_move_starts(last_value, new_value):
    return last_value != new_value and ((new_value <= STICK_END_THRESHOLD and STICK_END_THRESHOLD <= last_value) or (last_value <= -STICK_END_THRESHOLD and -STICK_END_THRESHOLD <= new_value))

def stick_move_ends(last_value, new_value):
    if (STICK_END_THRESHOLD <= last_value and STICK_END_THRESHOLD <= new_value) or (last_value <= -STICK_END_THRESHOLD and new_value <= -STICK_END_THRESHOLD):
        if math.floor(last_value * 1000) == math.floor(new_value * 1000):
            return True
    elif (STICK_THRESHOLD <= last_value and STICK_THRESHOLD <= new_value) or (last_value <= -STICK_THRESHOLD and new_value <= -STICK_THRESHOLD ):
        return True
    return False


def measure_main_loop(joystick, joystick_axis, ser, stats, response_curve_data, stop_event, change_event, reverse = False, reverse_from = 0):
    clock = pygame.time.Clock()

    move_count = reverse_from
    move_started = False
    no_move_count = 0
    move_start_count = 0
    move_end_count = 0
    direction = 0
    min_value = 1
    max_value = -1

    joystick_another_axis = "ry"
    if joystick_axis == "lx":
        joystick_another_axis = "ly"
    elif joystick_axis == "ly":
        joystick_another_axis = "lx"
    elif joystick_axis == "rx":
        joystick_another_axis = "ry"
    elif joystick_axis == "ry":
        joystick_another_axis = "rx"

    print(f'measure start from {move_count}, reverse mode: {reverse}.')
    ser.write(f"{move_count * DEFAULT_NUM_STEP}\n".encode())

    with tqdm(total=2.0, ncols=76, bar_format='{l_bar}{bar} | {postfix[0]}', dynamic_ncols=False, postfix=[0]) as pbar:
        start_time = time.perf_counter()
        while move_end_count == 0:
            current_start_time = time.perf_counter()
            cur_ms = pygame.time.get_ticks()

            quit_event = pygame.event.get(pygame.QUIT)
            if quit_event:
                stop_event.set()
                return
            
            joystick_remove_event = pygame.event.get(pygame.JOYDEVICEREMOVED)
            if joystick_remove_event:
                change_event.set()
                return

            if len(stats[joystick_axis]) == 0:
                measure_stats(joystick, stats, cur_ms, 0, 0)

            last_value = stats[joystick_axis][-1]

            # Moving the motor
            if reverse:
                move_count -= 1
            else:
                move_count += 1

            ser.write(f"{move_count * DEFAULT_NUM_STEP}\n".encode())

            pygame.time.wait(TIME_STICK_MOVEMENT_MS)


            # Get the time from pygame.init() called in ms.
            current_time = time.perf_counter()
            elapsed_time = (current_time - start_time) * 1000

            result = measure_stats(joystick, stats, cur_ms, elapsed_time, move_count * STEP_DISTANCE_MM)

            # Check stats if it moves or not
            new_value = result[joystick_axis]
            pbar.postfix[0] = "{:s}: {:05.3f}, {:s}: {:05.3f}".format(joystick_axis, new_value, joystick_another_axis, result[joystick_another_axis])

            if not move_started:
                if stick_move_starts(last_value, new_value):
                    # MOVE STARTS
                    move_started = True
                    move_start_count = move_count - 1
                    if new_value < last_value:
                        max_value = last_value
                        direction = -1
                        print(f'\nmove started at count {move_start_count}, direction is down/right to up/left.')
                    else:
                        min_value = last_value
                        direction = 1
                        print(f'\nmove started at count {move_start_count}, direction is up/left to down/right.')
                else:
                    no_move_count +=1
                pbar.update(0)
            else:
                if stick_move_ends(last_value, new_value):
                    # MOVE ENDS
                    no_move_count += 1
                    if direction < 0:
                        min_value = new_value
                        move_end_count = move_count
                        print(f'\nmove ended at count {move_end_count}, min value is {new_value}.')

                    elif 0 < direction:
                        max_value = new_value
                        move_end_count = move_count
                        print(f'\nmove ended at count {move_end_count}, max value is {max_value}.')
                    
                if (direction > 0):
                    pbar.update(new_value - last_value)
                elif (direction < 0):
                    pbar.update(last_value - new_value)

            # Wait until next measure frame
            clock.tick(MEASURE_FRAME_RATE)
    
    movement_stats = stats[joystick_axis][abs(reverse_from - move_start_count): abs(reverse_from - move_end_count)]

    calc_response_curve(movement_stats, direction, min_value, max_value, abs(move_end_count - move_start_count) * STEP_DISTANCE_MM, joystick_axis, response_curve_data, reverse)
    save_response_curve(response_curve_data)

    return move_end_count


def calc_response_curve(movement_stats, direction, min, max, distance, joystick_axis, response_curve_data, reverse = False):
    
    # Distance from 0 step to center
    def determine_center_from_zero_idx(movement_stats):
        most_minimum_plus_center = 1
        most_minimum_plus_center_idx = 0
        most_minimum_minus_center = -1
        most_minimum_minus_center_idx = 0

        for idx, val in enumerate(movement_stats):
            if (most_minimum_minus_center < val and val < 0):
                most_minimum_minus_center = val
                most_minimum_minus_center_idx = idx
            if (0 < val and val < most_minimum_plus_center):
                most_minimum_plus_center = val
                most_minimum_plus_center_idx = idx

        steps_between = most_minimum_minus_center_idx - most_minimum_plus_center_idx
        distance_between = steps_between * STEP_DISTANCE_MM

        center_distance_from_plus = (- distance_between * most_minimum_plus_center / (most_minimum_plus_center - most_minimum_minus_center))

        center_distance = most_minimum_plus_center_idx * STEP_DISTANCE_MM - center_distance_from_plus

        #print(f'len: {len(movement_stats)}, min minus: {most_minimum_minus_center}({most_minimum_minus_center_idx}), plus: {most_minimum_plus_center}({most_minimum_plus_center_idx}), center_distance_from_plus: {center_distance_from_plus}, center_distance: {center_distance}')

        return center_distance


    # Calculating Stick Degrees from Neutral
    def calc_degree(mm_from_zero):
        def calc_right_to_center_degree():
            dist_from_zero = abs(mm_from_zero)

            theta_plus_alpha = math.asin((dist_from_zero + STICK_RADIUS_MM ) / math.sqrt(STICK_HEIGHT_MM ** 2 + STICK_RADIUS_MM ** 2))
            alpha = math.asin(STICK_RADIUS_MM / math.sqrt(STICK_HEIGHT_MM ** 2 + STICK_RADIUS_MM ** 2))

            return math.degrees(theta_plus_alpha - alpha)
        
        def calc_left_to_center_degree():
            dist_from_zero = abs(mm_from_zero)

            theta_plus_alpha = math.asin((dist_from_zero - STICK_RADIUS_MM ) / math.sqrt(STICK_HEIGHT_MM ** 2 + STICK_RADIUS_MM ** 2))
            alpha = math.asin(- STICK_RADIUS_MM / math.sqrt(STICK_HEIGHT_MM ** 2 + STICK_RADIUS_MM ** 2))

            #print(f'dist_from_zero: {dist_from_zero}, theta_plus_alpha: {theta_plus_alpha}, alpha: {alpha}, degree: {theta_plus_alpha - alpha}, dist_from_zero_calc_by_theta: {STICK_HEIGHT_MM * math.sin(theta_plus_alpha - alpha) + STICK_RADIUS_MM / 2 - (STICK_RADIUS_MM / 2 * math.cos(theta_plus_alpha - alpha))}')

            return math.degrees(theta_plus_alpha - alpha)

        #print(f'mm_from_zero: {mm_from_zero}, direction: {direction}, reverse: {reverse}')

        degree = 0
        if (mm_from_zero < 0):
            if reverse: #left to center \ -> |
                degree = calc_left_to_center_degree()
            else:       #center to right | -> /
                degree = calc_right_to_center_degree()
        else:
            if reverse: #right to center | <- /
                degree = calc_right_to_center_degree()
            else:       #center to left \ <- |
                degree = calc_left_to_center_degree()
        
        if direction > 0:
            degree = - degree
        
        return degree


    # Calculating Distance of Stick Top-Center from Neutral Center Axis
    def calc_distance_from_neutral_center_axis(mm_from_zero, degree):
        distance_from_neutral_center_axis = 0
        if mm_from_zero <= 0:
            if reverse: #left to center \ -> |
                distance_from_neutral_center_axis = ((-mm_from_zero) + STICK_RADIUS_MM * math.cos(math.radians(degree)) - STICK_RADIUS_MM)
            else:       #center to right | -> /
                distance_from_neutral_center_axis = (-mm_from_zero) - STICK_RADIUS_MM * math.cos(math.radians(degree)) + STICK_RADIUS_MM
        else:
            if reverse: #right to center | <- /
                distance_from_neutral_center_axis = - (mm_from_zero - STICK_RADIUS_MM * math.cos(math.radians(degree)) + STICK_RADIUS_MM)
            else:       #center to left \ <- |
                distance_from_neutral_center_axis = - (mm_from_zero + STICK_RADIUS_MM * math.cos(math.radians(degree)) - STICK_RADIUS_MM)

        if direction > 0:
            distance_from_neutral_center_axis = - distance_from_neutral_center_axis
        
        return distance_from_neutral_center_axis

    distance_to_center = determine_center_from_zero_idx(movement_stats)
    print(f'distance to center from 0mm is {distance_to_center}mm')

    for idx, val in enumerate(movement_stats):
        step = idx
        step_distance = step * STEP_DISTANCE_MM

        mm_from_zero = step_distance - distance_to_center
        
        degree = calc_degree(mm_from_zero)

        compensated_dist_from_center = calc_distance_from_neutral_center_axis(mm_from_zero, degree)

        diff_for_degree = 0
        diff_for_dist = 0

        if (degree != 0):
            diff_for_degree = val / (degree / LINEAR_CURVE_MAX_DEGREE) - 1
        if (compensated_dist_from_center != 0):
            diff_for_dist = val / (compensated_dist_from_center / LINEAR_CURVE_CENTER_MAX_DISTANCE) - 1

        response_curve_data.append([degree, - mm_from_zero, val, diff_for_degree, compensated_dist_from_center, diff_for_dist])

    return response_curve_data


def save_response_curve(data):
    dt = datetime.datetime.now()
    filename = dt.strftime("%Y%m%d_%H%M%S_%f.csv")
    with open(filename, 'w') as fd:
        writer = csv.writer(fd)
        writer.writerows(data)


def visualization_main_loop(screen, stats, response_curve_data, joystick_axis, stop_event, change_event, reverse_stats = None, reverse_response_curve_data = None):
    clock = pygame.time.Clock()
    
    X=0; Y=1
    GRAPH_BOX_TOP_LEFT = (100, 50)
    GRAPH_BOX_WIDTH = 600
    GRAPH_BOX_HEIGHT = 400
    GRAPH_BOX_BOTTOM_RIGHT = (GRAPH_BOX_TOP_LEFT[X] + GRAPH_BOX_WIDTH, GRAPH_BOX_TOP_LEFT[Y] + GRAPH_BOX_HEIGHT)
    GRAPH_BOX_LINES = 20

    GRAPH_DIFF_BOX_TOP_LEFT = (GRAPH_BOX_TOP_LEFT[X], GRAPH_BOX_BOTTOM_RIGHT[Y] + 50)
    GRAPH_DIFF_BOX_HEIGHT = 100
    GRAPH_DIFF_BOX_BOTTOM_RIGHT = (GRAPH_DIFF_BOX_TOP_LEFT[X] + GRAPH_BOX_WIDTH, GRAPH_DIFF_BOX_TOP_LEFT[Y] + GRAPH_DIFF_BOX_HEIGHT)
    GRAPH_DIFF_BOX_LINES = 10

    GRAPH_MAIN_LINE_DELTA = 20

    SCREEN_COLOR = (60, 60, 60)
    GRAPH_MAIN_LINE_COLOR = (200, 200, 200)
    GRAPH_SUB_LINE_COLOR = (150, 150, 150)
    GRAPH_LINE_COLOR = (255, 128, 128)
    GRAPH_LINEAR_LINE_COLOR = (150, 150, 150)
    GRAPH_SECOND_LINE_COLOR = (128, 128, 255)

    graph_box = (GRAPH_BOX_TOP_LEFT[X], GRAPH_BOX_TOP_LEFT[Y], GRAPH_BOX_WIDTH, GRAPH_BOX_HEIGHT)

    def draw_stats(st_data, line_color = GRAPH_LINE_COLOR):
        stick_moved = False
        stick_moved_from = 0.0
        direction = 0
        max_distance = 14.0 #mm

        for idx, val in enumerate(st_data[joystick_axis][1:-1]):
            val_before = st_data[joystick_axis][idx - 1]
            distance = st_data["motor_pos"][idx]
            distance_before = st_data["motor_pos"][idx - 1]

            if max_distance < (distance - stick_moved_from):
                max_distance = distance - stick_moved_from

            if not stick_moved:
                if stick_move_starts(val_before, val):
                    stick_moved = True
                    stick_moved_from = distance
                    if 0 < val:
                        direction = -1
                    else:
                        direction = 1
                else: #(val <= -1.0 or 1.0 <= val)
                    continue

            if 0 < direction:
                pygame.draw.line(screen, line_color,
                    (
                        GRAPH_BOX_TOP_LEFT[X] + abs(GRAPH_BOX_WIDTH * (distance_before - stick_moved_from) / max_distance),
                        GRAPH_BOX_TOP_LEFT[Y] + GRAPH_BOX_HEIGHT - GRAPH_BOX_HEIGHT * ((val_before + 1.0) / 2.0)
                    ), (
                        GRAPH_BOX_TOP_LEFT[X] + abs(GRAPH_BOX_WIDTH * (distance - stick_moved_from) / max_distance),
                        GRAPH_BOX_TOP_LEFT[Y] + GRAPH_BOX_HEIGHT - GRAPH_BOX_HEIGHT * ((val + 1.0) / 2.0)
                ))
            else:
                pygame.draw.line(screen, line_color,
                    (
                        GRAPH_BOX_BOTTOM_RIGHT[X] - abs(GRAPH_BOX_WIDTH * (distance_before - stick_moved_from) / max_distance),
                        GRAPH_BOX_TOP_LEFT[Y] + GRAPH_BOX_HEIGHT - GRAPH_BOX_HEIGHT * ((val_before + 1.0) / 2.0)
                    ), (
                        GRAPH_BOX_BOTTOM_RIGHT[X] - abs(GRAPH_BOX_WIDTH * (distance - stick_moved_from) / max_distance),
                        GRAPH_BOX_TOP_LEFT[Y] + GRAPH_BOX_HEIGHT - GRAPH_BOX_HEIGHT * ((val + 1.0) / 2.0)
                ))


    def draw_response_curve_data(rc_data, linear_max, graph_max, key_idx, val_idx, line_color = GRAPH_LINE_COLOR):

        pygame.draw.line(screen, GRAPH_LINEAR_LINE_COLOR,
            (
                GRAPH_BOX_TOP_LEFT[X] + GRAPH_BOX_WIDTH / 2 + GRAPH_BOX_WIDTH * (-linear_max) / graph_max,
                GRAPH_BOX_TOP_LEFT[Y] + GRAPH_BOX_HEIGHT
            ),(
                GRAPH_BOX_TOP_LEFT[X] + GRAPH_BOX_WIDTH / 2 + GRAPH_BOX_WIDTH * linear_max / graph_max,
                GRAPH_BOX_TOP_LEFT[Y]
        ), 5)

        for idx, data in enumerate(rc_data[2:-1]):
            val = data[val_idx]
            val_before = rc_data[idx - 1 + 2][val_idx]
            key = data[key_idx]
            key_before = rc_data[idx - 1 + 2][key_idx]

            pygame.draw.line(screen, line_color,
                (
                    GRAPH_BOX_TOP_LEFT[X] + GRAPH_BOX_WIDTH / 2 + GRAPH_BOX_WIDTH * key_before / graph_max,
                    GRAPH_BOX_TOP_LEFT[Y] + GRAPH_BOX_HEIGHT - GRAPH_BOX_HEIGHT * ((val_before + 1.0) / 2.0)
                ), (
                    GRAPH_BOX_TOP_LEFT[X] + GRAPH_BOX_WIDTH / 2 + GRAPH_BOX_WIDTH * key / graph_max,
                    GRAPH_BOX_TOP_LEFT[Y] + GRAPH_BOX_HEIGHT - GRAPH_BOX_HEIGHT * ((val + 1.0) / 2.0)
            ), 2)
        
            #DIFF
            if (key != 0 and key_before != 0):
                diff_before = val_before / (key_before / linear_max) - 1
                diff = val / (key / linear_max) - 1

                pygame.draw.line(screen, line_color,
                    (
                        GRAPH_DIFF_BOX_TOP_LEFT[X] + GRAPH_BOX_WIDTH / 2 + GRAPH_BOX_WIDTH * key_before / graph_max,
                        GRAPH_DIFF_BOX_TOP_LEFT[Y] + GRAPH_DIFF_BOX_HEIGHT / 2 - GRAPH_DIFF_BOX_HEIGHT * diff_before
                    ), (
                        GRAPH_DIFF_BOX_TOP_LEFT[X] + GRAPH_BOX_WIDTH / 2 + GRAPH_BOX_WIDTH * key / graph_max,
                        GRAPH_DIFF_BOX_TOP_LEFT[Y] + GRAPH_DIFF_BOX_HEIGHT / 2 - GRAPH_DIFF_BOX_HEIGHT * diff
                ), 2)

    def draw_response_curve_data_by_center_distance(rc_data, line_color = GRAPH_LINE_COLOR):
        draw_response_curve_data(rc_data, LINEAR_CURVE_CENTER_MAX_DISTANCE, 18.0, 4, 2, line_color)

    def draw_response_curve_data_by_distance(rc_data, line_color = GRAPH_LINE_COLOR):
        draw_response_curve_data(rc_data, LINEAR_CURVE_MAX_DISTANCE, 18.0, 4, 1, line_color)

    def draw_response_curve_data_by_degree(rc_data, line_color = GRAPH_LINE_COLOR):
        draw_response_curve_data(rc_data, LINEAR_CURVE_MAX_DEGREE, 45.0, 0, 2, line_color)


    while not stop_event.is_set() and not change_event.is_set():
        screen.fill(SCREEN_COLOR)

        ## Response Curve Graph
        # Drawing Scales
        for i in range(1, GRAPH_BOX_LINES, 1):
            line_x = GRAPH_BOX_TOP_LEFT[X] + i * GRAPH_BOX_WIDTH / GRAPH_BOX_LINES
            pygame.draw.line(screen, GRAPH_SUB_LINE_COLOR, (line_x, GRAPH_BOX_TOP_LEFT[Y]), (line_x, GRAPH_BOX_BOTTOM_RIGHT[Y]), 1)
            line_y = GRAPH_BOX_TOP_LEFT[Y] + i * GRAPH_BOX_HEIGHT / GRAPH_BOX_LINES
            pygame.draw.line(screen, GRAPH_SUB_LINE_COLOR, (GRAPH_BOX_TOP_LEFT[X], line_y), (GRAPH_BOX_BOTTOM_RIGHT[X], line_y), 1)

        # Drawing Axes
        line_x = GRAPH_BOX_TOP_LEFT[X] + GRAPH_BOX_WIDTH / 2
        pygame.draw.line(screen, GRAPH_MAIN_LINE_COLOR, (line_x, GRAPH_BOX_TOP_LEFT[Y] - GRAPH_MAIN_LINE_DELTA), (line_x, GRAPH_BOX_BOTTOM_RIGHT[Y] + GRAPH_MAIN_LINE_DELTA), 2)
        line_y = GRAPH_BOX_TOP_LEFT[Y] + GRAPH_BOX_HEIGHT / 2
        pygame.draw.line(screen, GRAPH_MAIN_LINE_COLOR, (GRAPH_BOX_TOP_LEFT[X] - GRAPH_MAIN_LINE_DELTA, line_y), (GRAPH_BOX_BOTTOM_RIGHT[X] + GRAPH_MAIN_LINE_DELTA, line_y), 2)

        ## Difference to Linear Graph
        # Drawing Scales
        for i in range(1, GRAPH_DIFF_BOX_LINES, 1):
            line_x = GRAPH_DIFF_BOX_TOP_LEFT[X] + i * GRAPH_BOX_WIDTH / GRAPH_DIFF_BOX_LINES
            pygame.draw.line(screen, GRAPH_SUB_LINE_COLOR, (line_x, GRAPH_DIFF_BOX_TOP_LEFT[Y]), (line_x, GRAPH_DIFF_BOX_BOTTOM_RIGHT[Y]), 1)
            line_y = GRAPH_DIFF_BOX_TOP_LEFT[Y] + i * GRAPH_DIFF_BOX_HEIGHT / GRAPH_DIFF_BOX_LINES
            pygame.draw.line(screen, GRAPH_SUB_LINE_COLOR, (GRAPH_DIFF_BOX_TOP_LEFT[X], line_y), (GRAPH_DIFF_BOX_BOTTOM_RIGHT[X], line_y), 1)

        # Drawing Axes
        line_x = GRAPH_DIFF_BOX_TOP_LEFT[X] + GRAPH_BOX_WIDTH / 2
        pygame.draw.line(screen, GRAPH_MAIN_LINE_COLOR, (line_x, GRAPH_DIFF_BOX_TOP_LEFT[Y] - GRAPH_MAIN_LINE_DELTA), (line_x, GRAPH_DIFF_BOX_BOTTOM_RIGHT[Y] + GRAPH_MAIN_LINE_DELTA), 2)
        line_y = GRAPH_DIFF_BOX_TOP_LEFT[Y] + GRAPH_DIFF_BOX_HEIGHT / 2
        pygame.draw.line(screen, GRAPH_MAIN_LINE_COLOR, (GRAPH_DIFF_BOX_TOP_LEFT[X] - GRAPH_MAIN_LINE_DELTA, line_y), (GRAPH_DIFF_BOX_BOTTOM_RIGHT[X] + GRAPH_MAIN_LINE_DELTA, line_y), 2)


        ## Drawing Graphs
        if len(response_curve_data) > 2:
            draw_response_curve_data_by_center_distance(response_curve_data)
        else:
            draw_stats(stats)

        if len(reverse_response_curve_data) > 2:
            draw_response_curve_data_by_center_distance(reverse_response_curve_data, GRAPH_SECOND_LINE_COLOR)
        else:
            draw_stats(reverse_stats, GRAPH_SECOND_LINE_COLOR)


        pygame.display.flip()
        clock.tick(60)


def start_main_loop(screen, joystick, joystick_axis, ser, stop_event, change_event):
    def gen_stats():
        stats = {}
        for key in ["timestamps", "elapsed_time", "motor_pos", "lx", "ly", "rx", "ry", "lt", "rt"]:
            stats[key] = []
        return stats

    def gen_response_curve_data():
        response_curve_data = [['degrees', 'distances', 'values', 'diff_degrees', 'compensated_distances', 'diff_compensated_distances']]
        return response_curve_data

    # Preparing Variables
    stats = gen_stats()
    response_curve_data = gen_response_curve_data()
    reverse_stats = gen_stats()
    reverse_response_curve_data = gen_response_curve_data()

    # Starting Visualization Thread
    visualization_thread = None
    visualization_thread = Thread(target=visualization_main_loop, args=(screen, stats, response_curve_data, joystick_axis, stop_event, change_event, reverse_stats, reverse_response_curve_data))
    visualization_thread.start()


    # Starting Measurement
    move_end_count = measure_main_loop(joystick, joystick_axis, ser, stats, response_curve_data, stop_event, change_event)

    if REVERSE_MODE:
        measure_main_loop(joystick, joystick_axis, ser, reverse_stats, reverse_response_curve_data, stop_event, change_event, True, move_end_count)

    # Resetting Motor Position
    ser.write("0\n".encode())


    input("Press Enter to finish.")

    if visualization_thread:
        stop_event.set()
        visualization_thread.join()
        stop_event.clear()


def main():
    os.system('cls' if os.name == 'nt' else 'clear')
    os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"    #get key events while the window is not focused
    
    stop_event = Event()
    change_event = Event()

    while True:
        pygame.init()


        try:
            # Parsing Args and Preparing Variables
            args = parse_args()

            comports = list_ports.comports()
            joystick_count = pygame.joystick.get_count()

            if args.list: #list com ports and controllers

                print(" -- com ports -- ")
                for i in range(len(comports)):
                    com_port = comports[i]
                    print(f"{i}: {com_port.description}")

                print("\n -- controllers -- ")
                if joystick_count > 0:
                    for i in range(joystick_count):
                        joystick = pygame.joystick.Joystick(i)
                        print(f"{i}: {joystick.get_name()}")
                
                input("Press Enter to Exit.")

                return

            com_port_idx = args.com_port
            controller_idx = args.controller
            joystick_axis = args.axis


            # Validations 
            if len(comports) == 0:
                print(f"\033[31mSerial device Not Found.\033[0m")
                time.sleep(5)
                return
            if len(comports) <= com_port_idx:
                print(f"\033[31mCOM port index number must be less than {len(comports)} but {com_port_idx}.\033[0m")
                time.sleep(5)
                return
            if joystick_count <= controller_idx:
                print(f"\033[31mController index number must be less than {joystick_count} but {controller_idx}.\033[0m")
                time.sleep(5)
                return
            
            if not (joystick_axis == "lx" or joystick_axis == "ly" or joystick_axis == "rx" or joystick_axis == "ry"):
                print("Invalid joystick axis specified. Defaulting to the right x.")
                joystick_axis = "rx"


            # Starting Measurement
            ser = serial.Serial(comports[com_port_idx].device, DEFAULT_SERIAL_BAUD_RATE)

            try:
                joystick = pygame.joystick.Joystick(controller_idx)
                screen = pygame.display.set_mode(WINDOW_SIZE)
                pygame.display.set_caption(WINDOW_CAPTION)

                start_main_loop(screen, joystick, joystick_axis, ser, stop_event, change_event)

            finally:
                ser.close()

        finally:
            pygame.quit()
            stop_event.clear()
            change_event.clear()


        input("Press Enter to run again")



if __name__ == "__main__":

    main()