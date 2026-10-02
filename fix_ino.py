with open('rover_qa/qa_01_border_v2/qa_01_border_v2.ino', 'r') as f:
    text = f.read()

text = text.replace('Serial.println("  [6] Mision MULTI-CUBO AUTONOMA (Busca 2 cubos por cercania)");\\n    Serial.println("  [r] Detener motores");',
                    'Serial.println("  [6] Mision MULTI-CUBO AUTONOMA (Busca 2 cubos por cercania)");\n    Serial.println("  [r] Detener motores");')

with open('rover_qa/qa_01_border_v2/qa_01_border_v2.ino', 'w') as f:
    f.write(text)
