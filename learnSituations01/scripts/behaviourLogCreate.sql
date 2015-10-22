DROP TABLE IF EXISTS `BehaviourLog`; 
CREATE TABLE `BehaviourLog` (
  `timestamp` datetime NOT NULL COMMENT 'Date and time reading was received',
  `sensorId` int(11) NOT NULL COMMENT 'sensor Id',
  `status` text NOT NULL COMMENT 'sensor status	'
) ENGINE=MyISAM DEFAULT CHARSET=latin1 COMMENT='logs sensor data based pn user behaviour.';
