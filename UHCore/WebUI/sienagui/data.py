from Data.dataAccess import DataAccess
from config import siena_config, server_config
import json


__encoding = None


def encoding():
    global __encoding
    if __encoding == None:
        dao = DataAccess()
        sql = "SELECT \
                   default_character_set_name\
               FROM\
                   information_schema.SCHEMATA S\
               WHERE\
                   schema_name = %(schema)s"

        args = {'schema': server_config['mysql_log_db']}
        result = dao.sql.getSingle(sql, args)
        if result:
            __encoding = result['default_character_set_name']
        else:
            __encoding = 'latin1'
    return __encoding


class Login(object):
    exposed = True

    def __init__(self):
        self._dao = DataAccess()

    def GET(self, unick=''):

        user = self._dao.users.getUserByNickName(unick)
        if user == None:
            return None

        self._dao.users.setActiveUser(user['userId'])

        ret = "%(userId)s,%(languageId)s" % {'userId': user['userId'], 'languageId': user['languageId']}
        return ret


class Options(object):
    exposed = True

    def __init__(self):
        self._dao = DataAccess()

    def GET(self, ulang='-1', pid='-1'):

        sql = "SELECT \
                a.idActionPossibilityOptions AS id, \
                a.OptionName AS name, \
                a.PossibleValues AS 'values', \
                a.DefaultValue AS 'default' \
            FROM \
                ActionPossibilityOptions a INNER JOIN\
                ActionPossibility_APOptions o ON a.idActionPossibilityOptions = o.idOpt \
            WHERE \
                o.idAP = %(sonid)s"

        args = {'sonid': pid}

        results = self._dao.sql.getData(sql, args)
        return json.dumps(results, encoding=encoding())


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

        sql = " \
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
            return json.dumps(result['expression'], encoding=encoding())


class FullActionList(object):
    exposed = True

    def __init__(self):
        self._dao = DataAccess()
        self._likelihood = siena_config['likelihood']

    def GET(self, session='1'):

        sql = " \
            SELECT\
                lt.message as ap_label,\
                p.message as phraseal_feedback,\
                apt.text as type_description,\
                ap.likelihood,\
                ap.apId,\
                ap.precondId as precond_id,\
                ap.x as x,\
                ap.y as y\
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
        return json.dumps(results, encoding=encoding())


class RobotActions(object):
    exposed = True

    def __init__(self):
        self._dao = DataAccess()
        self._likelihood = siena_config['likelihood']

    def GET(self, session='1', ulang=None, robot=None, uid=None):

        sql = " \
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

        args = {'threshold': self._likelihood, 'session': session}

        results = self._dao.sql.getData(sql, args)
        return json.dumps(results, encoding=encoding())


class SonsActions(object):
    exposed = True

    def __init__(self):
        self._dao = DataAccess()
        self._likelihood = siena_config['likelihood']

    def GET(self, session='1', ulang=None, pid='-1'):

        sql = " \
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

        args = {'threshold': self._likelihood, 'session':session, 'parent': pid}

        results = self._dao.sql.getData(sql, args)
        return json.dumps(results, encoding=encoding())


class UserActions(object):
    exposed = True

    def __init__(self):
        self._dao = DataAccess()
        self._likelihood = siena_config['likelihood']

    def GET(self, session='1', uid=None, ulang=None):

        sql = " \
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

        args = {'threshold': self._likelihood, 'session': session}

        results = self._dao.sql.getData(sql, args)
        return json.dumps(results, encoding=encoding())


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

        args = {'value': val, 'optId': opt_id}

        self._dao.sql.saveData(sql, args)
        return "OK"


if __name__ == '__main__':
    print encoding()
    print ExpressionRequest().GET()
    print FullActionList().GET()
    print Login().GET()
    print Options().GET()
    print RobotActions().GET()
    print SetParameter().GET()
    print SonsActions().GET()
    print UserActions().GET()
