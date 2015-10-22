drop table if exists ruleStateHistory;

-- create a new table of states excluding the start state row

create TABLE ruleStateHistory 
SELECT * FROM SensorStateHistory where lastUpdate > '2012-01-01 00:00:01';

drop table if exists ruleTimings;

-- this computes the duration and elpased times

CREATE TABLE ruleTimings AS
select 
      (TIMEDIFF(S.lastUpdate,@lasttm)) as duration,
      (TIMEDIFF(S.lastUpdate,@starttm)) as relativeTime,
      @lasttm := S.lastUpdate as actualTime
   from
      ruleStateHistory S,
      ( select @starttm := lastUpdate, @lasttm := lastUpdate from ruleStateHistory order by lastUpdate limit 1) StartRow 
   order by
     S.lastUpdate;

-- update the new state table

update ruleStateHistory S, ruleTimings R set S.duration = R.duration, S.relativeTime = R.relativeTime where S.lastUpdate = R.actualTime;  
