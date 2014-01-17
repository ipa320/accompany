import handlers, os
from linkData import LinkData

name = "Site Root"
links = {}

root = handlers.Index(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'dirIndex.html'))
root.js = handlers.StaticFiles(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'js'))
root.css = handlers.StaticFiles(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'css'))
root.favicon_ico = handlers.StaticFile(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'favicon.ico'))

def setLinks(links):
    for (path, webRoot) in links:
        if type(webRoot) != str:
            root.__dict__[path] = webRoot.root

    root.js.links = LinkData(links)
    return root