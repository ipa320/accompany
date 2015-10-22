
drop table if exists TeachingLog;
CREATE TABLE `TeachingLog` (
  `relativeTime` time NOT NULL,
  `class` text NOT NULL
) ENGINE=MyISAM DEFAULT CHARSET=latin1 COMMENT='Holds teaching information for show me application';
