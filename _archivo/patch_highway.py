import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # We will replace the steering block in DRIVE_AVOID, DRIVE_PRE, SENSOR_APPROACH, DRIVE_PUSH
    old_steer = """                float correction = 0.0f;
                if (fabs(diff) > ANGLE_DEADBAND_DEG) correction = diff * KP_STEERING;
                correction = constrain(correction, -0.12f, 0.12f);
                setMotors(current_cruise - correction, current_cruise + correction);"""
                
    new_steer = """                float correction = 0.0f;
                // MODO CARRETERA (Trazos largos): Menos correcciones para evitar zig-zag
                float dyn_deadband = (dist > 12.0f) ? 8.0f : ANGLE_DEADBAND_DEG;
                float dyn_kp = (dist > 12.0f) ? (KP_STEERING * 0.4f) : KP_STEERING;
                
                if (fabs(diff) > dyn_deadband) correction = diff * dyn_kp;
                correction = constrain(correction, -0.12f, 0.12f);
                setMotors(current_cruise - correction, current_cruise + correction);"""
                
    content = content.replace(old_steer, new_steer)

    with open(filepath, 'w') as f:
        f.write(content)

for rid in [1, 2]:
    patch(f"rover_qa/rover_{rid}/src/navigation.cpp")
print("Highway mode patched")
