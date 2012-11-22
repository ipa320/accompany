server_config = {
  # The port on which the web server will be running
  # Point your web browser to http://localhost:<http_port>
  # must be above 104 to avoid root privilage!!

  'http_port':       1054,

  # The settings for the channel logging MySQL server / database / table
  'mysql_log_server':   'rh-database',
  #'mysql_log_server':   'localhost',
  'mysql_log_user':     'rhUser',
  'mysql_log_password': 'waterloo',
  'mysql_log_db':       'Accompany',
  'mysql_log_table':    'SensorLog',
  
  'mysql_history_table':'ActionHistory',
  'mysql_sensorHistory_table':'SensorHistory',
  'mysql_sensorType_table':'SensorType',
  'mysql_sensor_table':'Sensors',
  'mysql_location_table':'Locations',
  'mysql_robot_table':'Robot',
  'mysql_image_table':'Images',
  'mysql_questions_table':'userInterfaceGUI',
  'mysql_responses_table':'userInterfaceGUI',
  'mysql_users_table':'Users',
  
    # The port on which the program is listening for UDP broadcast messages
  # transmitted by the ZigBee gateway

  'udp_listen_port': 5050,

  # The settings of the Geo-System MySQL server / database / table
  'mysql_geo_server':   'geo-eee-pc',
  'mysql_geo_user':     'guest',
  'mysql_geo_password': 'r0b0th0use##',
  'mysql_geo_db':       'livewiredb',
  'mysql_geo_query':    'CALL expPower',
}
