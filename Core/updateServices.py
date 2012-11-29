import locations
import sensors
from history import SensorLog
from Robots.careobot import CareOBot, PoseUpdater

from config import server_config
import sys

if __name__ == '__main__':
    robot = CareOBot()
    z = sensors.ZigBee(server_config['udp_listen_port'])
    g = sensors.GEOSystem(server_config['mysql_geo_server'],
                            server_config['mysql_geo_user'],
                            server_config['mysql_geo_password'],
                            server_config['mysql_geo_db'],
                            server_config['mysql_geo_query'])
    l = locations.RobotLocationProcessor(robot)
    rp = PoseUpdater(robot)

    sz = SensorLog(z.channels, 'ZigBee')
    sg = SensorLog(g.channels, 'GEO')
    sr = SensorLog(rp.channels, rp.robot.name)

    z.start()
    sz.start()

    g.start()
    sg.start()

    rp.start()
    sr.start()
    
    l.start()
    
    while True:
        try:
            sys.stdin.read()
        except KeyboardInterrupt:
            break
    l.stop()

    sr.stop()
    rp.stop()

    sg.stop()
    g.stop()

    sz.stop()
    z.stop()
