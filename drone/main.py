"""
Core script for starting all subsystems for Marker Search competition
"""

import numpy as np
import os
import sys
from time import sleep, strftime
import argparse
import logging
from queue import Queue
from threading import Event
from pprint import pprint

import drone_globals as gl
from pilot import PilotThread
from camera import CameraThread

parser = argparse.ArgumentParser(description='Starts onboard systems')
# Logging
parser.add_argument('-l', '--log', dest='loglevel', type=str.upper,
                    choices=['CRITICAL', 'ERROR', 'WARNING', 'INFO', 'DEBUG'],
                    help='Select logging level to console case-insensitive, default DEBUG', default='DEBUG')
# Selecting run environment
parser.add_argument('-s', '--sitl', action='store_const', dest='sitl_on', const=True,
                    help='Launching with SITL, default off', default=False)
parser.add_argument('-c', dest='camera_id', type=int,
                    help='Select camera id for capture, default 3', default=3)
# Threads on/off
parser.add_argument('--camera', dest='camera', action='store_true')
parser.add_argument('--no-camera', dest='camera', action='store_false')
parser.set_defaults(camera=True)

try:
    args = parser.parse_args()
except argparse.ArgumentError as e:
    print e.message
    exit()
gl.args = args

numeric_level = getattr(logging, args.loglevel.upper(), None)
if not isinstance(numeric_level, int):
    raise ValueError('Invalid log level: {}'.format(args.loglevel))

if not os.path.exists(gl.logs_dir):
    os.makedirs(gl.logs_dir)

datefmt = '%Y.%m.%d_%H.%M.%S'
logfilename = gl.logs_dir + 'log_' + strftime(datefmt) + ".txt"
logfmt = logging.Formatter('%(asctime)s %(levelname)s:\t%(module)s %(funcName)s() %(message)s', datefmt=datefmt)

log = logging.getLogger()
log.setLevel(logging.DEBUG)

fh = logging.FileHandler(logfilename)
fh.setLevel(logging.DEBUG)
fh.setFormatter(logfmt)
ch = logging.StreamHandler(sys.stdout)
ch.setLevel(numeric_level)
ch.setFormatter(logfmt)

log.addHandler(fh)
log.addHandler(ch)

main_running = True


def main():
    # Log all argparse options
    log.info('Starting main with arguments ' + str(args)[10:-1])

    gl.takeoff_event = Event()

    log.info('Initialising pilot')
    gl.pilot = PilotThread()
    gl.pilot.start()

    if args.camera:
        log.info('Initialising camera')
        gl.camera = CameraThread()
        gl.camera.start()

    gl.takeoff_event.wait()
    gl.pilot.goto_position_target_local_ned(0,-2.5,-gl.alt)
    gl.pilot.set_roi(80, 20)
    sleep(4)
    print 'pilot in position'

    # gl.pilot.goto_position_target_local_ned(0, 2.5, -gl.alt)
    # gl.pilot.set_roi(80, 20)

    pointlist = []
    collect = True
    while collect:
        pointlist.append(list(gl.pilot.position_ned()))
        sleep(0.1)

    pprint(pointlist)

    global main_running
    while main_running:
        # Main loop
        sleep(3)


def quitmain():
    log.debug('Stopping main')

    global main_running
    main_running = False

    if args.camera:
        log.debug('Stopping camera')
        try:
            gl.camera.stop()
            gl.camera.join()
        except BaseException as ex:
            log.error('Error stopping kalman: ' + str(ex))

    log.debug('Stopping pilot')
    try:
        gl.pilot.stop()
        gl.pilot.join()
    except BaseException as ex:
        log.error('Error stopping pilot: ' + str(ex))


# App entry point:
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt, SystemExit:
        quitmain()
        exit()
