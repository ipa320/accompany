from Data.dataAccess import DataAccess
import time

class Current(object):
    exposed = True
    
    def __init__(self):
        self._dao = DataAccess()
        
    def POST(self):
        
        sql="SELECT value FROM Sensors WHERE sensorId=%(sid)s"
        args = {'sid': 1001 }
        
        result = self._dao.sql.getSingle(sql, args)
        
        if result == None:
            return "error"
        else:
            value=result['value']
            sql="SELECT actionName FROM RobotActionsHistory ORDER BY timestamp DESC LIMIT 1;"
            result = self._dao.sql.getSingle(sql)
            if result == None:
                return "error"
            else:
                return "%s,%s" % (value, result['actionName'])

class Next(object):
    exposed = True
    
    def __init__(self):
        self._dao = DataAccess()
        
    def POST(self, current='-1',task=''):
        status = current
        start_status=current
        start_task=task.strip()
        count = 60  #rough timeout of about 15sec --> if nothing changes in 15 sec the response will be empty
        
        #Do...while pattern, exit condition at the bottom
        while True:
            sql="SELECT value FROM Sensors WHERE sensorId=%(sid)s"
            args = {'sid': 1001 }
            
            result = self._dao.sql.getSingle(sql, args)
            
            if result == None:
                print 'Problem! rowcount=0'
                return "error"

            status=str(result['value'])
    
            sql="SELECT actionName FROM RobotActionsHistory ORDER BY timestamp DESC LIMIT 1;"
            result = self._dao.sql.getSingle(sql)
            if result == None:
                return "error"
            else:
                task=result['actionName']
            
            if status == start_status and task == start_task and count >= 0:
                time.sleep(0.25)
                count-=1
            else:
                break

        if status == start_status and task == start_task:
            return ""
        else:
            return "%s,%s" % (status, task)
