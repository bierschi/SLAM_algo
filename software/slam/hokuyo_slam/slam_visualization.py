# Robot display params
ROBOT_HEIGHT_MM = 500
ROBOT_WIDTH_MM = 300

# This helps with Raspberry Pi
import matplotlib

matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
import matplotlib.cm as colormap
from math import sin, cos, radians
import numpy as np
from time import sleep
import threading
import array
import random
import sys


class SlamShow(object):

    def __init__(self, map_size_pixels, map_scale_mm_per_pixel, title):

        # Store constants for update
        self.map_size_pixels = map_size_pixels
        self.map_scale_mm_per_pixel = map_scale_mm_per_pixel

        # Create a byte array to display the map with a color overlay
        self.bgrbytes = bytearray(map_size_pixels * map_size_pixels * 3)

        # Make a nice big (10"x10") figure
        fig = plt.figure(figsize=(10, 10))

        # Store Python ID of figure to detect window close
        self.figid = id(fig)

        fig.canvas.set_window_title('SLAM')
        plt.title(title)

        self.ax = fig.gca()
        self.ax.set_aspect("auto")
        self.ax.set_autoscale_on(True)

        # Use an "artist" to speed up map drawing
        self.img_artist = None

        # We base the axis on pixels, to support displaying the map
        self.ax.set_xlim([0, map_size_pixels])
        self.ax.set_ylim([0, map_size_pixels])

        # Hence we must relabel the axis ticks to show millimeters
        ticks = np.arange(0, self.map_size_pixels + 100, 100)
        labels = [str(self.map_scale_mm_per_pixel * tick) for tick in ticks]
        self.ax.xaxis.set_ticks(ticks)
        self.ax.set_xticklabels(labels)
        self.ax.yaxis.set_ticks(ticks)
        self.ax.set_yticklabels(labels)

        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')

        self.ax.grid(False)

        # Start vehicle at center
        map_center_mm = map_scale_mm_per_pixel * map_size_pixels
        self._add_vehicle(map_center_mm, map_center_mm, 0)

        self.toggle_thread = threading.Thread(target=self.toggle_flag)
        self.toggle_thread.start()
        self.i = 0
        self.image_flag = True

    def displayMap(self, mapbytes):

        mapimg = np.reshape(np.frombuffer(mapbytes, dtype=np.uint8), (self.map_size_pixels, self.map_size_pixels))

        # Pause to allow display to refresh
        sleep(.001)

        #with open("mapbytes.txt", "a") as file:
        #    file.write("{}, {}\n".format(len(list(bytes(mapbytes))), list(bytes(mapbytes))))

        if self.img_artist is None:

            self.img_artist = self.ax.imshow(mapimg, cmap=colormap.gray)

        else:

            self.img_artist.set_data(mapimg)

    def save_image(self):

        if self.image_flag is True:
            plt.savefig("images/slam{}.png".format(self.i))
            self.i += 1
            self.image_flag = False

    def save_pgm(self, mapbytes):

        if self.image_flag is True:

            width = self.map_size_pixels
            height = self.map_size_pixels

            filename = 'slam{}.pgm'.format(self.i)

            self.i += 1
            try:
                fout = open(filename, 'wb')
            except IOError:
                print("cannot open file")
                sys.exit()

            pgmHeader = 'P2' + '\n' + str(width) + ' ' + str(height) + ' ' + str(255) + '\n'
            pgmHeader = bytearray(pgmHeader, 'utf-8')
            fout.write(pgmHeader)
            mapimg = np.reshape(np.frombuffer(mapbytes, dtype=np.uint8), (self.map_size_pixels, self.map_size_pixels))
            for j in range(height):
                bnd = list(mapimg[j, :])
                bnd_str = np.char.mod('%d', bnd)
                bnd_str = np.append(bnd_str, '\n')
                bnd_str = [' '.join(bnd_str)][0]
                bnd_byte = bytearray(bnd_str, 'utf-8')
                fout.write(bytearray(bnd_byte))
            fout.close()
            self.image_flag = False

    def toggle_flag(self):

        while True:
            sleep(2)
            self.image_flag = not self.image_flag

    def setPose(self, x_mm, y_mm, theta_deg):
        '''
        Sets vehicle pose:
        X:      left/right   (cm)
        Y:      forward/back (cm)
        theta:  rotation (degrees)
        '''
        # remove old arrow
        self.vehicle.remove()

        # create a new arrow
        self._add_vehicle(x_mm, y_mm, theta_deg)

    def _add_vehicle(self, x_mm, y_mm, theta_deg):

        # Use a very short arrow shaft to orient the head of the arrow
        dx, dy = plt_rotate(0, 0, 0.1, theta_deg)

        s = self.map_scale_mm_per_pixel

        self.vehicle = self.ax.arrow(x_mm / s, y_mm / s,
                                     dx, dy, head_width=ROBOT_WIDTH_MM / s, head_length=ROBOT_HEIGHT_MM / s, fc='r',
                                     ec='r')

    def refresh(self):

        # If we have a new figure, something went wrong (closing figure failed)
        if self.figid != id(plt.gcf()):
            return False

        # Redraw current objects without blocking
        plt.draw()

        # Refresh display, setting flag on window close or keyboard interrupt
        try:
            plt.pause(.01)  # Arbitrary pause to force redraw
            return True
        except:
            return False

        return True

    # Converts millimeters to pixels
    def mm2pix(self, mm):
        return int(mm / float(self.map_scale_mm_per_pixel))


# Helpers -------------------------------------------------------------

def plt_rotate(x, y, r, deg):
    rad = radians(deg)
    c = cos(rad)
    s = sin(rad)
    dx = r * c
    dy = r * s
    return x + dx, y + dy

