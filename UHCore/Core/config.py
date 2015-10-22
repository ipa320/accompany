""" Generic web and database configurations """
""" TODO: Sensor configurations should be moved into the locations_config element as they can differ between sites """
server_config = {
  # The port on which the web server will be running
  # Point your web browser to http://localhost:<http_port>
  # must be above 104 to avoid root privilege!!
  'http_port':       1055,

  # The settings for the channel logging MySQL server / database / table
  'mysql_log_server':   'localhost',#'rh-database',
  'mysql_log_user':     'rhUser',
  'mysql_log_password': 'waterloo',
  'mysql_log_db':       'AccompanyResources',
  'mysql_log_table':    'SensorLog',
  
  'mysql_history_table':'ActionHistory',
  'mysql_scenario_table':'Scenarios',
  'mysql_sensorHistory_table':'SensorHistory',
  'mysql_sensorType_table':'SensorType',
  'mysql_sensor_table':'Sensors',
  'mysql_location_table':'Locations',
  'mysql_robot_table':'Robot',
  'mysql_image_table':'Images',
  'mysql_imageoverhead_table':'ImagesOverhead',
  'mysql_questions_table':'userInterfaceGUI',
  'mysql_responses_table':'userInterfaceGUI',
  'mysql_users_table':'Users',
  'mysql_user_preferences_table': 'UserPreferences',
  'mysql_personas_table': 'Personas',
  'mysql_experimentLocations_table':'ExperimentalLocation',
  'mysql_session_control_table':'SessionControl',
  'mysql_sensorIcon_table': 'SensorIcons',
  
  # The port on which the program is listening for UDP broadcast messages
  # transmitted by the ZigBee gateway
  'udp_listen_port': 5000,
  'zigbee_usb_port': '/dev/ttyUSB0',

  # The settings of the Geo-System MySQL server / database / table
  'mysql_geo_server':   'geo-eee-pc',
  'mysql_geo_user':     'guest',
  'mysql_geo_password': 'r0b0th0use##',
  'mysql_geo_db':       'livewiredb',
  'mysql_geo_query':    'CALL expPower',
  
  # Settings for the zwave sensor network,
  'zwave_ip': '192.168.1.109',
}

""" Contains configuration information for each experiment site """
""" The 'sensors' element controlls which sensors classes are loaded when sensors.py is run """
""" The 'map' element is used to control the conversion between map coordinates and svg image coordinates """

locations_config = {
  'ZUYD Apartment': {
                     'sensors': ['ZWaveHomeController', 'ZigBeeDirect'],
                     'map': {
                                 'base':'zuyd.svg',
                                 'scale':0.275,
                                 'offset':(105, 300),
                                 'rotation':-90 
                            }
                     },
  'UH Robot House': {
#                     'sensors': ['ZigBee', 'GEOSystem', 'ZigBeeDirect'],
                     'sensors': ['ZigBee', 'GEOSystem'],
                     'map': {
                                'base':'RobotHouseMap.svg',
                                'scale':0.275,
                                'offset':(81, 245),
                                'rotation':-90 
                            }
                     },
  'STRI e125': {
#                     'sensors': ['ZigBee', 'ZigBeeDirect'],
                     'sensors': ['ZigBee'],
                     'map': {
                                'base':'RobotHouseMap.svg',
                                'scale':0.275,
                                'offset':(81, 245),
                                'rotation':-90 
                            }
                     },
}

""" Controlls various magic strings that are specific to individulal robot models
    Structure is as follows:
        'Robot Name': {
                        'componentName': {
                                            'positions': {
                                                            'generic': 'robot specific rosparam',
                                                         }
                                            ...other component specific settings...
                                         }
                     }
    Some component settings are: 
        tray:size==Activation range for the phidget sensors (COB Only)
        head:camera:topic==ros topic to get the image from
        head:camera:rotate: {
                                'angle': when to rotate the image
                                'distance': tolerance (angle+-distance)
                                'amount': how much to rotate by (img.rotate(amount))
                            }
        hostname:override hostname constructed by robotFactory()
            I.E. COB3.2 default is built as cob3-2-pc1, if hostname is set, it will be used instead
    
"""
robot_config = {
                'Care-O-Bot 3.2': {
                                   'phidgets': { 'topics': ['/range_0', '/range_1', '/range_2', '/range_3'], 'windowSize': 5 },
                                   'tray': { 
                                                'positions': { 
                                                              'raised': 'up',
                                                              'lowered': 'down' 
                                                              },
                                                'size' : 20
                                            },
                                   'head': { 
                                                'positions': { 'front': 'front', 'back': 'back' },
                                                'camera': {
                                                       'topic':'/stereo/right/image_color/compressed',
                                                       'rotate': {
                                                                  'angle': 180,
                                                                  'distance': 90,
                                                                  'amount': 180
                                                                 }
                                                       }
                                            },
                                   },
                'Care-O-Bot 3.5': {
                                   'phidgets': {'topics': ['/tray_sensors/range_0', '/tray_sensors/range_1', '/tray_sensors/range_2', '/tray_sensors/range_3'], 'windowSize': 5 },
                                   'tray': { 
                                                'positions': {
                                                              'raised': 'deliverup',
                                                              'lowered': 'storetray',
                                                              },
                                                'size' : 10
                                            },
                                   'head': { 
                                                'positions': { 'front': 'front', 'back': 'back' },
                                                'camera': {
                                                       'topic':'/stereo/right/image_color/compressed',
                                                       'rotate': {
                                                                         'angle': 0,
                                                                         'distance': 90,
                                                                         'amount': 180
                                                                         }
                                                       }
                                            },
                                   },
                'Care-O-Bot 3.6': {
                                   'phidgets': {'topics': ['/tray_sensors/range_0', '/tray_sensors/range_1', '/tray_sensors/range_2', '/tray_sensors/range_3'], 'windowSize': 5 },
                                   'tray': { 
                                                'positions': {
                                                              'raised': 'deliverup',
                                                              'raised': 'home',
                                                              'lowered': 'store'
                                                              },
                                                'size' : 10

                                            },
                                   'head': { 
                                                'positions': { 'front': 'front', 'back': 'back' },
                                                'camera': {
                                                       'topic':'/stereo/right/image_color/compressed',
                                                       'rotate': {
                                                                         'angle': 0,
                                                                         'distance': 90,
                                                                         'amount': 180
                                                                         }
                                                       }
                                            },
                                   },
                'Sunflower 1-1': {
                                   'hostname': 'sf1-1-pc1',
                                   'tray': { 
                                                'positions': { 
                                                              'raised': 'open', 
                                                              'lowered': 'closed',
                                                              }
                                            },
                                   'head': { 
                                                'positions': { 
                                                              'front': 'home', 'back': 'back_right' 
                                                              }
                                            },
                                   },
                'Sunflower 1-2': {
                                   'tray': { 
                                                'positions': { 
                                                              'raised': 'open', 'lowered': 'closed' 
                                                              }
                                            },
                                   'head': { 
                                                'positions': { 
                                                              'front': 'home', 'back': 'back_right' 
                                                              }
                                            },
                                   },
                }

""" Threshold level used to indicate when to display an action possibility on the siena gui """
siena_config = {
    'likelihood': 0.1
}

