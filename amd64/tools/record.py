#!/usr/bin/env python3

"""BSD 2-Clause License

Copyright (c) 2019, Allied Vision Technologies GmbH
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import sys
import os
import time
import threading
from signal import signal, SIGINT
from typing import Optional
from cv2 import cv2
import numpy
from pynput.keyboard import Listener, Key, KeyCode
from datetime import datetime
from collections import deque
from getch import pause

from vimba import (Vimba, Frame, Camera, FrameStatus, intersect_pixel_formats,
                   COLOR_PIXEL_FORMATS, MONO_PIXEL_FORMATS, VimbaCameraError,
                   VimbaFeatureError, PersistType)

ESC_KEY = 27
SETTINGS_PREFIX = os.path.dirname(os.path.realpath(__file__)) + "/"
SETTINGS_FILE = SETTINGS_PREFIX + "Mako_G-158C_settings.xml"
#VIDEO_PREFIX = os.environ['HOME'] + "/Desktop/videos/"
#VIDEO_FILE = VIDEO_PREFIX + "test" + datetime.now().strftime("%Y_%H_%M_%S") + ".avi"
VIDEO_FILE = "test" + datetime.now().strftime("%Y_%H_%M_%S") + ".avi"
CAM_FPS = 30.0
VIDEO = deque()
TOTAL_FRAMES = 0
TERMINATING = False
PREVIEW = False

def print_usage():
    """Print help information."""
    print('Program for previewing and saving frames')
    print('Usage:')
    print('    python [camera_id]')
    print('    python  [/h] [-h]')
    print()
    print('Parameters:')
    print('    camera_id   ID of the camera to use (using first camera if not specified)')
    print('Press \'p\' key to turn ON/OFF preview window. This feature may slow down capture rate!')
    print('Press \'ESC\' key to stop the recording script.')
    print()

def graceful_exit_handler(signal_received, frame):
    """Exiting helper function, 'frame' adds debugging info."""
    print("SIGINT or CTRL-C detected. Exiting gracefully.", frame)
    cv2.destroyAllWindows()
    sys.exit(signal_received)

def abort(reason: str, return_code: int = 1, usage: bool = False):
    """Abort execution and print reason."""
    print(reason + '\n')

    if usage:
        print_usage()

    pause("Press any key to exit")
    sys.exit(return_code)

def parse_args() -> Optional[str]:
    """Parse command line arguments."""
    args = sys.argv[1:]
    argc = len(args)

    for arg in args:
        if arg in ('/h', '-h'):
            print_usage()
            sys.exit(0)

    if argc > 1:
        abort(reason="Invalid number of arguments. Abort.", return_code=2, usage=True)

    return None if argc == 0 else args[0]

def get_camera(cam_id: Optional[str]):
    """Get camera if it was specified otherwise use first detected camera."""
    with Vimba.get_instance() as vimba:
        if cam_id:
            try:
                return vimba.get_camera_by_id(cam_id)

            except VimbaCameraError:
                abort('Failed to access Camera {}. Abort.'.format(cam_id))
        else:
            cams = vimba.get_all_cameras()
            if not cams:
                abort('No Camera detected. Abort.')

            return cams[0]

def setup_camera(cam: Camera):
    """Try to setup camera parameters."""
    with cam:
        # Try to adjust GeV packet size. This Feature is only available for GigE - Cameras.
        try:
            cam.GVSPAdjustPacketSize.run()
            while not cam.GVSPAdjustPacketSize.is_done():
                pass

            # Load camera settings from file.
            cam.load_settings(SETTINGS_FILE, PersistType.All)
            print(f'Camera settings have been loaded from file: {SETTINGS_FILE}')
            cam.AcquisitionFrameRateAbs.set(CAM_FPS);
            fps = cam.AcquisitionFrameRateAbs.get();
            width = cam.Width.get();
            height = cam.Height.get();
            print(f'Camera configured to fps: [{fps}] and resolution: [{width}x{height}]')


        except Exception as err:
            abort(f'Failed to load camera settings reson: {err}. Aborting.')

def resize(img: numpy.ndarray) -> numpy.ndarray:
    """Helper function resizes the given image to fit smaller screen's preview."""
    return cv2.resize(img, (640, 480), interpolation=cv2.INTER_AREA)


class Handler():
    """Handler handles acquired frames, displays them, saves visible frame, and
       terminates on ESC key or CTRL-C."""
    def __init__(self, event):
        self.msg = 'Stream from the {} camera. Press <ESC> to stop stream.'
        self.shutdown_event = event


    def __call__(self, cam: Camera, frame: Frame):
        if ESC_KEY == cv2.waitKey(1) or not threading.main_thread().is_alive():
            self.shutdown_event.set()
        # NOTE below commented line prints every time frame is captured
        # print('{} acquired {}'.format(cam, frame), flush=True)
        elif frame.get_status() == FrameStatus.Complete:
            image = frame.as_numpy_ndarray()
            image = cv2.cvtColor(image, cv2.COLOR_BayerBG2BGR)
            VIDEO.append(image)
            global PREVIEW
            if PREVIEW:
                cv2.imshow(self.msg.format(cam.get_name()), resize(image))
            else:
                cv2.destroyAllWindows()

        if self.shutdown_event.isSet():
            cv2.destroyAllWindows()
        else:
            cam.queue_frame(frame)

def on_press(key):
    """Monitor keys and act on action keys pressed"""
    global TERMINATE
    try:
        if key == Key.esc:
            print("\nESC key pressed - stopping recording script.")
            global TERMINATING
            TERMINATING = True
        elif key.char == 'p':
            global PREVIEW
            if PREVIEW:
                print("\nTurning OFF preview window.")
                PREVIEW = False
            else:
                print("\nTurning ON preview window - May slow down fps capture rate!")
                PREVIEW = True
    except AttributeError:
        pass

def on_release(key):
    """Do nothing on key release"""
    pass

class Capture(threading.Thread):
    """Stream and preview video frames."""
    cam_id = parse_args()

    with Vimba.get_instance():
        with get_camera(cam_id) as cam:
            # Try to configure camera
            setup_camera(cam)

            # Setup keyboard listener
            listener = Listener(on_press=on_press, on_release=on_release)
            listener.daemon=True
            listener.start()

	    # Pass shutdown threading event to the child thread
            shutdown_event = threading.Event()
            handler = Handler(shutdown_event)

	    # Configure video writer
            fps = cam.AcquisitionFrameRateAbs.get();
            res = (cam.Width.get(), cam.Height.get());
            fourcc = cv2.VideoWriter_fourcc(*'DIVX')
            out = cv2.VideoWriter(VIDEO_FILE, fourcc, fps, res)

            print("\n###########################################################")
            print(f'Recording stream from the {cam.get_name()} camera. Press <ESC> to stop.')
            print('\nPress \'p\' key to turn ON/OFF preview window.')
            print('This feature may slow down capture rate!')
            print("###########################################################\n")
            # Measure time for debugging purposes
            start_time = time.time()
            try:
                # Start Streaming with default buffer size of 5 frames
                cam.start_streaming(handler=handler)
                while not handler.shutdown_event.isSet() or len(VIDEO):
                    global TERMINATING
                    if TERMINATING:
                        shutdown_event.set()
                    if len(VIDEO):
                        out.write(VIDEO[0])
                        TOTAL_FRAMES += 1
                        VIDEO.pop()

                out.release()
                print(f'Successfully recorded video: [{VIDEO_FILE}]')
                rec_time = time.time() - start_time
                print(f'Recorded for [{rec_time}], Total captured frames: [{TOTAL_FRAMES}], what gives [{TOTAL_FRAMES / rec_time}] fps.')

            finally:
                cam.stop_streaming()

if __name__ == '__main__':
    signal(SIGINT, graceful_exit_handler)
    cap = Capture()
    cap.start()
    # Main thread waits for cap to finish
    cap.join()
    abort(f'Finished...')
