with open("central_brain.py", "r") as f:
    content = f.read()

import re
new_log = '''def web_log(tag, msg):
    try:
        import urllib.request
        import json
        data = json.dumps({"tag": tag, "msg": msg}).encode('utf-8')
        req = urllib.request.Request('http://127.0.0.1:8891/api/log', data=data, headers={'Content-Type': 'application/json'})
        urllib.request.urlopen(req, timeout=0.1)
    except:
        pass'''

content = re.sub(r'def web_log\(tag, msg\):.*?pass', new_log, content, flags=re.DOTALL)

with open("central_brain.py", "w") as f:
    f.write(content)
