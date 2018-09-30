from breezylidar import URG04LX as Lidar
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import URG04LX as LaserModel
from time import sleep
from slam_visualization import SlamShow

MAP_SIZE_PIXELS = 500
MAP_SIZE_METERS = 10


def main():

    lidar = Lidar(device='/dev/ttyACM0')
    slam = RMHC_SLAM(LaserModel(), map_size_pixels=MAP_SIZE_PIXELS, map_size_meters=MAP_SIZE_METERS)

    display = SlamShow(MAP_SIZE_PIXELS, MAP_SIZE_METERS*1000/MAP_SIZE_PIXELS, 'SLAM')

    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    while True:
        slam.update(lidar.getScan())
        x, y, theta = slam.getpos()
        #print(x, y, theta)
        print(mapbytes)
        slam.getmap(mapbytes)
        display.displayMap(mapbytes)
        display.setPose(x, y, theta)

        if not display.refresh():
            exit(0)


if __name__ == '__main__':
    main()
