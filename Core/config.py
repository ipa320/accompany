server_config = {
  # The port on which the web server will be running
  # Point your web browser to http://localhost:<http_port>
  # must be above 104 to avoid root privilage!!

  'http_port':       1054,

  # The settings for the channel logging MySQL server / database / table
  'mysql_log_server':   'rh-database',
  'mysql_log_user':     'rhUser',
  'mysql_log_password': 'waterloo',
  'mysql_log_db':       'Accompany',
  'mysql_log_table':    'SensorLog',
  
  'mysql_history_table':'ActionHistory',
  'mysql_sensorHistory_table':'SensorHistory',
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


################################################################################
#
# The configuration of the ZigBee devices and channels
#
################################################################################
zigbee_devices = {
  '[00:13:a2:00:40:32:de:87]!' : {      # ZigBee device MAC address
    'room': 'Kitchen',                  # Room name
    'AD0': {                            # Channel descriptor
      'id'  : 1,                        # This ID is only relevant for logging
      'name': 'Water pipe Hot',         # Channel name
      'type': 'TEMPERATURE_MCP9700_HOT' # Channel type
    },
    'AD1': {
      'id'  : 2,
      'name': 'Water pipe Cold',
      'type': 'TEMPERATURE_MCP9700_COLD'
    },
    'DIO2': {
      'id'  : 3,
      'name': 'Ceiling cupboard door left',
      'type': 'CONTACT_REED'
    },
    'DIO3': {
      'id'  : 4,
      'name': 'Ceiling cupboard door middle',
      'type': 'CONTACT_REED'
    },
    'DIO4': {
      'id'  : 5,
      'name': 'Ceiling cupboard door right',
      'type': 'CONTACT_REED'
    },
    'DIO5': {
      'id'  : 6,
      'name': 'Floor cupboard drawer middle',
      'type': 'CONTACT_REED'
    },
    'DIO7': {
      'id'  : 8,
      'name': 'Floor cupboard door middle',
      'type': 'CONTACT_REED'
    },
    'DIO10': {
      'id'  : 10,
      'name': 'Floor cupboard door left',
      'type': 'CONTACT_REED'
    },
    'DIO11': {
      'id'  : 7,
      'name': 'Floor cupboard drawer right',
      'type': 'CONTACT_REED'
    },
    'DIO12': {
      'id'  : 9,
      'name': 'Floor cupboard door right',
      'type': 'CONTACT_REED'
    }
  },

  '[00:13:a2:00:40:62:a9:52]!' : {
    'room': 'Bathroom',
    'AD0': {
      'id'  : 11,
      'name': 'Water pipe Hot',
      'type': 'TEMPERATURE_MCP9700_HOT'
    },
    'AD1': {
      'id'  : 12,
      'name': 'Water pipe Cold',
      'type': 'TEMPERATURE_MCP9700_COLD'
    },
    'DIO2': {
      'id'  : 13,
      'name': 'Bathroom Door',
      'type': 'CONTACT_REED'
    },
    'DIO3': {
      'id'  : 14,
      'name': 'Toilet Flush',
      'type': 'CONTACT_REED'
    }
  },

  '[00:13:a2:00:40:62:a9:4f]!' : {
    'room': 'Bedroom',
    'DIO0': {
      'id'  : 32,
      'name': 'Desk drawer bottom',
      'type': 'CONTACT_REED'
    },
    'DIO1': {
      'id'  : 33,
      'name': 'Desk drawer middle',
      'type': 'CONTACT_REED'
    },
    'DIO2': {
      'id'  : 34,
      'name': 'Desk drawer top',
      'type': 'CONTACT_REED'
    },
    'DIO3': {
      'id'  : 35,
      'name': 'Desk door',
      'type': 'CONTACT_REED'
    },
    'DIO4': {
      'id'  : 36,
      'name': 'Office chair',
      'type': 'CONTACT_PRESSUREMAT'
    },
    'DIO5': {
      'id'  : 37,
      'name': 'Bedroom door',
      'type': 'CONTACT_REED'
    },
    'DIO7': {
      'id'  : 39,
      'name': 'Bed contact',
      'type': 'CONTACT_PRESSUREMAT'
    },
    'DIO10': {
      'id'  : 41,
      'name': 'Wardrobe door left',
      'type': 'CONTACT_REED'
    },
    'DIO11': {
      'id'  : 42,
      'name': 'Wardrobe door middle',
      'type': 'CONTACT_REED'
    },
    'DIO12': {
      'id'  : 43,
      'name': 'Wardrobe door right',
      'type': 'CONTACT_REED'
    }
  },

  '[00:13:a2:00:40:62:a9:4d]!' : {
    'room': 'Living room (Sofa)',
    'DIO0': {
      'id'  : 15,
      'name': 'Seatplace 0',
      'type': 'CONTACT_PRESSUREMAT'
    },
    'DIO1': {
      'id'  : 16,
      'name': 'Seatplace 1',
      'type': 'CONTACT_PRESSUREMAT'
    },
    'DIO2': {
      'id'  : 17,
      'name': 'Seatplace 2',
      'type': 'CONTACT_PRESSUREMAT'
    },
    'DIO3': {
      'id'  : 18,
      'name': 'Seatplace 3',
      'type': 'CONTACT_PRESSUREMAT'
    },
    'DIO4': {
      'id'  : 19,
      'name': 'Seatplace 4',
      'type': 'CONTACT_PRESSUREMAT'
    }
  },

  '[00:13:a2:00:40:62:a9:4e]!' : {
    'room': 'Living room (Table)',
    'DIO0': {
      'id'  : 20,
      'name': 'Seatplace 0',
      'type': 'CONTACT_PRESSUREMAT'
    },
    'DIO1': {
      'id'  : 21,
      'name': 'Seatplace 1',
      'type': 'CONTACT_PRESSUREMAT'
    },
    'DIO2': {
      'id'  : 22,
      'name': 'Seatplace 2',
      'type': 'CONTACT_PRESSUREMAT'
    },
    'DIO4': {
      'id'  : 24,
      'name': 'Living room door',
      'type': 'CONTACT_REED'
    },
    'DIO5': {
      'id'  : 25,
      'name': 'Cupboard big drawer bottom',
      'type': 'CONTACT_REED'
    },
    'DIO3': {
      'id'  : 26,
      'name': 'Cupboard big drawer top',
      'type': 'CONTACT_REED'
    },
    'DIO7': {
      'id'  : 27,
      'name': 'Cupboard small door left',
      'type': 'CONTACT_REED'
    },
    'DIO6': {
      'id'  : 28,
      'name': 'Cupboard small door right',
      'type': 'CONTACT_REED'
    },
    'DIO10': {
      'id'  : 29,
      'name': 'Cupboard small drawer bottom',
      'type': 'CONTACT_REED'
    },
    'DIO11': {
      'id'  : 30,
      'name': 'Cupboard small drawer middle',
      'type': 'CONTACT_REED'
    },
    'DIO12': {
      'id'  : 31,
      'name': 'Cupboard small drawer top',
      'type': 'CONTACT_REED'
    }
  }
}

################################################################################
#
# The configuration of the Geo-System channels
#
################################################################################
geosystem_devices = {
  1: {                         # The "channel ID" delivered by the MySQL function
    'id'  : 44,                # This ID is only relevant for logging
    'room': 'Other',           # Room name
    'name': 'Lights exterior', # Channel name
    'rule': 'p > 10'           # Device is "On", if power consumption is higher than 10 Watts
  },
  2: {
    'id'  : 45,
    'room': 'Other',
    'name': 'Upstairs Lights',
    'rule': 'p > 10'
  },
  3: {
    'id'  : 46,
    'room': 'Other',
    'name': 'Downstairs Lights',
    'rule': 'p > 10'
  },
  7: {
    'id'  : 47,
    'room': 'Kitchen',
    'name': 'Cooker',
    'rule': 'p > 10'
  },
  8: {
    'id'  : 48,
    'room': 'Other',
    'name': 'Garage',
    'rule': 'p > 5'
  },
  9: {
    'id'  : 49,
    'room': 'Other',
    'name': 'Sockets',
    'rule': 'p > 5'
  },
  10: {
    'id'  : 50,
    'room': 'Other',
    'name': 'Sockets Ext. and garden',
    'rule': 'p > 5'
  },
  12: {
    'id'  : 51,
    'room': 'Other',
    'name': 'Mains Supply',
    'rule': 'p > 10'
  },
  13: {
    'id'  : 52,
    'room': 'Living room (Sofa)',
    'name': '1 TV',
    'rule': 'p > 10'
  },
  14: {
    'id'  : 53,
    'room': 'Kitchen',
    'name': '2 Fridge Freezer',
    'rule': '(p > 10 and p < 50) or (p > 100)'
  },
  15: {
    'id'  : 54,
    'room': 'Kitchen',
    'name': '3 Kettle',
    'rule': 'p > 10'
  },
  16: {
    'id'  : 55,
    'room': 'Bedroom',
    'name': '4 Computer',
    'rule': 'p > 10'
  },
  17: {
    'id'  : 56,
    'room': 'Bedroom',
    'name': '5 Table Lamp',
    'rule': 'p > 10'
  },
  18: {
    'id'  : 57,
    'room': 'Kitchen',
    'name': '6 Microwave',
    'rule': 'p > 10'
  },
  19: {
    'id'  : 58,
    'room': 'Kitchen',
    'name': '7 Dishwasher',
    'rule': 'p > 10'
  },
  20: {
    'id'  : 59,
    'room': 'Kitchen',
    'name': 'Toaster',
    'rule': 'p > 10'
  },
  21: {
    'id'  : 60,
    'room': 'Living room (Sofa)',
    'name': '9 Living room light',
    'rule': 'p > 5'
  },
  22: {
    'id'  : 61,
    'room': 'Other',
    'name': '10 Rumba',
    'rule': 'p > 10'
  },
  24: {
    'id'  : 62,
    'room': 'Other',
    'name': '12 Doorbell',
    'rule': 'p > 1'
  }
}

### EOF ###