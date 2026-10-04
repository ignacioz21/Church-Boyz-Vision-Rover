import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # Find the line drawing part
    old_draw = """                    ctx.beginPath();
                    ctx.moveTo(r.p1[0] * scaleX, r.p1[1] * scaleY);
                    ctx.lineTo(r.p2[0] * scaleX, r.p2[1] * scaleY);
                    ctx.lineTo(r.p3[0] * scaleX, r.p3[1] * scaleY);
                    ctx.stroke();"""
                    
    new_draw = """                    ctx.beginPath();
                    ctx.moveTo(r.p1[0] * scaleX, r.p1[1] * scaleY);
                    if (r.p_avoid) {
                        ctx.lineTo(r.p_avoid[0] * scaleX, r.p_avoid[1] * scaleY);
                    }
                    ctx.lineTo(r.p2[0] * scaleX, r.p2[1] * scaleY);
                    ctx.lineTo(r.p3[0] * scaleX, r.p3[1] * scaleY);
                    ctx.stroke();
                    
                    if (r.p_avoid) {
                        ctx.fillStyle = color;
                        ctx.beginPath();
                        ctx.arc(r.p_avoid[0] * scaleX, r.p_avoid[1] * scaleY, 4, 0, Math.PI*2);
                        ctx.fill();
                    }"""
                    
    content = content.replace(old_draw, new_draw)

    with open(filepath, 'w') as f:
        f.write(content)

patch("dashboard/index.html")
print("UI patched for Avoidance Waypoint")
