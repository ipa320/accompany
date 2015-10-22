
-- You need to update apparmor to allow sql writes!
-- sudo gedit /etc/apparmor.d/usr.sbin.mysqld     
-- add the path/to/folder
-- sudo /etc/init.d/apparmor restart


SELECT * FROM ruleStateHistory
INTO OUTFILE '/home/joe/Documents/ACCOMPANY/Ismael/houseLearner/user001/Inferences.data'
  FIELDS TERMINATED BY ','  
  LINES TERMINATED BY '\n'
