import handlers, os
from actionQuestions import Data

name = "Active Questions"

root = handlers.Index(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'question.html'))
root.data = Data()
root.js = handlers.StaticFiles(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'js'))
root.css = handlers.StaticFiles(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'css'))
