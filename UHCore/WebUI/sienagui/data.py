from Data.dataAccess import DataAccess
from config import siena_config
import json

class Login(object):
    exposed = True
    
    def __init__(self):
        self._dao = DataAccess()
        
    def GET(self, unick=''):

        user = self._dao.users.getUserByNickName(unick)
        if user == None:
            return None

        sessionId = self._dao.users.setActiveUser(user['userId'])
                
        ret = "%(userId)s,%(languageId)s" % { 'userId':user['userId'], 'languageId':user['languageId'] }
        return ret

class Options(object):
    exposed = True
    
    def __init__(self):
        self._dao = DataAccess()
        
    def GET(self, ulang='-1', pid='-1'):

        sql="SELECT \
                a.idActionPossibilityOptions AS id, \
                a.OptionName AS name, \
                a.PossibleValues AS 'values', \
                a.DefaultValue AS 'default' \
            FROM \
                ActionPossibilityOptions a INNER JOIN\
                ActionPossibility_APOptions o ON a.idActionPossibilityOptions = o.idOpt \
            WHERE \
                o.idAP = %(sonid)s"

        args = {'sonid': pid }

        results = self._dao.sql.getData(sql, args)
        return json.dumps(results)

class Command(object):
    exposed = True
    
    def __init__(self):
        self._dao = DataAccess()
        
    def GET(self, cmd_id):
        
        if self._dao.sensors.saveSensorLog(cmd_id, True, 'Pressed'):
            return "OK"
        
        return "ERROR"
        
class ExpressionRequest(object):
    exposed = True
    
    def __init__(self):
        self._dao = DataAccess()
        
    def GET(self, cmd_id='-1'):

        sql=" \
            SELECT \
                expression \
            FROM \
                GUIexpression \
            WHERE \
                ison='1' \
            LIMIT \
                1"

        result = self._dao.sql.getSingle(sql)
        if result == None:
            return "error"
        else:
            return result['expression']

class FullActionList(object):
    exposed = True
    
    def __init__(self):
        self._dao = DataAccess()
        self._likelihood = siena_config['likelihood']
        
    def GET(self, session='1'):

        sql=" \
            SELECT\
                lt.message as ap_label,\
                p.message as phraseal_feedback,\
                apt.text as type_description,\
                ap.likelihood,\
                ap.apId,\
                ap.precondId as precond_id,\
                l.name as location_name\
            FROM\
                SessionControl s\
                INNER JOIN Users u ON u.userId = s.SessionUser\
                INNER JOIN ActionPossibilities ap\
                INNER JOIN ActionPossibilityType apt ON ap.apTypeId = apt.apTypeId\
                LEFT OUTER JOIN Locations l ON ap.locationId = l.locationId\
                INNER JOIN Messages lt ON ap.ap_text = lt.messageId AND u.languageId = lt.languageId\
                INNER JOIN Messages p ON ap.ap_phrase = p.messageId AND u.languageId = p.languageId\
            WHERE\
                s.sessionId = %(session)s AND\
                ap.parentId IS NULL AND\
                ap.likelihood > %(threshold)s"
                
        args = {'threshold': self._likelihood, 'session': session }

        results = self._dao.sql.getData(sql, args)
        return json.dumps(results)

class RobotActions(object):
    exposed = True
    
    def __init__(self):
        self._dao = DataAccess()
        self._likelihood = siena_config['likelihood']
        
    def GET(self, session='1', ulang=None, robot=None, uid=None):

        sql=" \
            SELECT\
                lt.message as ap_label,\
                p.message as phraseal_feedback,\
                apt.text as type_description,\
                ap.likelihood,\
                ap.apId,\
                ap.precondId as precond_id\
            FROM\
                SessionControl s\
                INNER JOIN Users u ON u.userId = s.SessionUser\
                INNER JOIN ExperimentalLocation e ON s.ExperimentalLocationId = e.id\
                INNER JOIN Robot r ON r.robotId = e.activeRobot\
                INNER JOIN ActionPossibilities ap ON ap.locationId = r.locationId OR ap.locationID IS NULL\
                INNER JOIN ActionPossibilityType apt ON ap.apTypeId = apt.apTypeId\
                INNER JOIN Messages lt ON ap.ap_text = lt.messageId AND u.languageId = lt.languageId\
                INNER JOIN Messages p ON ap.ap_phrase = p.messageId AND u.languageId = p.languageId\
            WHERE\
                s.sessionId = %(session)s AND\
                ap.parentId IS NULL AND\
                ap.likelihood > %(threshold)s\
            ORDER BY\
                ap.likelihood\
            DESC"
        
        args = {'threshold': self._likelihood, 'session':session }

        results = self._dao.sql.getData(sql, args)
        return json.dumps(results)

class SonsActions(object):
    exposed = True
    
    def __init__(self):
        self._dao = DataAccess()
        self._likelihood = siena_config['likelihood']
        
    def GET(self, session='1', ulang=None, pid='-1'):

        sql=" \
            SELECT\
                lt.message as ap_label,\
                p.message as phraseal_feedback,\
                apt.text as type_description,\
                ap.likelihood,\
                ap.apId,\
                ap.precondId as precond_id\
            FROM\
                SessionControl s\
                INNER JOIN Users u ON u.userId = s.SessionUser\
                INNER JOIN ActionPossibilities ap\
                INNER JOIN ActionPossibilityType apt ON ap.apTypeId = apt.apTypeId\
                INNER JOIN Messages lt ON ap.ap_text = lt.messageId AND u.languageId = lt.languageId\
                INNER JOIN Messages p ON ap.ap_phrase = p.messageId AND u.languageId = p.languageId\
            WHERE\
                s.sessionId = %(session)s AND\
                ap.parentId = %(parent)s AND\
                ap.likelihood > %(threshold)s\
            ORDER BY\
                ap.likelihood\
            DESC"
        
        args = {'threshold': self._likelihood, 'session':session, 'parent': pid }

        results = self._dao.sql.getData(sql, args)
        return json.dumps(results)

class UserActions(object):
    exposed = True
    
    def __init__(self):
        self._dao = DataAccess()
        self._likelihood = siena_config['likelihood']
        
    def GET(self, session='1', uid=None, ulang=None):

        sql=" \
            SELECT\
                lt.message as ap_label,\
                p.message as phraseal_feedback,\
                apt.text as type_description,\
                ap.likelihood,\
                ap.apId,\
                ap.precondId as precond_id\
            FROM\
                SessionControl s\
                INNER JOIN Users u ON u.userId = s.SessionUser\
                INNER JOIN ActionPossibilities ap ON ap.locationId = u.locationId OR ap.locationID IS NULL\
                INNER JOIN ActionPossibilityType apt ON ap.apTypeId = apt.apTypeId\
                INNER JOIN Messages lt ON ap.ap_text = lt.messageId AND u.languageId = lt.languageId\
                INNER JOIN Messages p ON ap.ap_phrase = p.messageId AND u.languageId = p.languageId\
            WHERE\
                s.sessionId = %(session)s AND\
                ap.parentId IS NULL AND\
                ap.likelihood > %(threshold)s\
            ORDER BY\
                ap.likelihood\
            DESC"
        
        args = {'threshold': self._likelihood, 'session': session }

        results = self._dao.sql.getData(sql, args)
        return json.dumps(results)

class SetParameter(object):
    exposed = True
    
    def __init__(self):
        self._dao = DataAccess()
        
    def GET(self, opt_id='-1', val='-1'):

        sql = " \
            UPDATE \
                ActionPossibilityOptions \
            SET \
                SelectedValue = %(value)s \
            WHERE \
                idActionPossibilityOptions = %(optId)s"

        args = { 'value': val, 'optId': opt_id }

        self._dao.sql.saveData(sql, args)
        return "OK"
