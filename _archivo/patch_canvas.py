import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # Change canvas background
    old_bg = "ctx.fillStyle = '#111';"
    new_bg = "ctx.fillStyle = '#fff';"
    content = content.replace(old_bg, new_bg)
    
    # Change text color
    old_txt = "ctx.fillStyle = '#fff';\n                    ctx.font = '10px Arial';"
    new_txt = "ctx.fillStyle = '#000';\n                    ctx.font = 'bold 10px Arial';"
    content = content.replace(old_txt, new_txt)
    
    # Grid lines or border if any... none implemented right now.

    with open(filepath, 'w') as f:
        f.write(content)

patch("dashboard/index.html")
print("Canvas patched for light mode")
