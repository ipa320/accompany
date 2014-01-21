import os
from data import Command, ExpressionRequest, FullActionList, Login, Options, RobotActions, SetParameter, SonsActions, UserActions
from control import Current, Next

name = "Siena GUI api"
__dir = os.path.dirname(os.path.realpath(__file__))

root = Login()
root.login = Login()
root.command = Command()
root.full_action_list = FullActionList() 
root.user_actions = UserActions()
root.robot_actions = RobotActions()
root.sons_actions = SonsActions()
root.expression_request = ExpressionRequest()
root.options =  Options()
root.setparameter =  SetParameter()
root.next = Next()
root.current = Current()