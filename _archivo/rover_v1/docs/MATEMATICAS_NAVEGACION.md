# Matemáticas de Navegación y Geometría del Rover

Este documento establece la formulación matemática rigurosa, el álgebra vectorial 2D y los algoritmos de control implementados en el firmware del rover para el **Vision Rover Challenge**.

---

## 1. Sistema de Coordenadas y Dimensiones

El sistema oficial de visión proyecta el campo en una cuadrícula continua normalizada en **celdas**:

* **Eje X (Columnas / $col$):** De izquierda a derecha ($0.0$ a $50.0$).
* **Eje Y (Filas / $row$):** De arriba hacia abajo ($0.0$ a $40.0$).
* **Factor de Escala:** $1.0\text{ celda} = 20\text{ mm} = 2.0\text{ cm}$.
* **Lado del Cubo:** $s_{cubo} = 50\text{ mm} = 2.5\text{ celdas}$. Radio equivalente de contorno: $r_{cubo} \approx 1.25\text{ celdas}$.
* **Orientación ($\theta$):** Ángulo en grados sexagesimales donde $0^\circ$ apunta hacia el eje $+X$ (derecha), $90^\circ$ apunta hacia $+Y$ (abajo), $180^\circ / -180^\circ$ hacia la izquierda y $-90^\circ$ hacia arriba.

---

## 2. Álgebra Vectorial y Puntos de Navegación

Sean:
* $\vec{P}_{rover} = (x_r, y_r)$ la posición actual del rover.
* $\vec{P}_{cubo} = (x_c, y_c)$ la posición actual del centro del cubo objetivo.
* $\vec{P}_{depot} = (x_d, y_d)$ la coordenada del centro de la zona de acopio correspondiente al color del cubo.

### 2.1. Vector de Empuje Unitario ($\vec{u}_{push}$)
El empuje óptimo debe desplazar el cubo directamente hacia el centro de su depósito:

$$\vec{v}_{push} = \vec{P}_{depot} - \vec{P}_{cubo} = (x_d - x_c, \; y_d - y_c)$$

$$\|\vec{v}_{push}\| = \sqrt{(x_d - x_c)^2 + (y_d - y_c)^2}$$

$$\vec{u}_{push} = \frac{\vec{v}_{push}}{\|\vec{v}_{push}\|} = \left(\frac{x_d - x_c}{\|\vec{v}_{push}\|}, \; \frac{y_d - y_c}{\|\vec{v}_{push}\|}\right)$$

El ángulo objetivo de empuje es:

$$\theta_{push} = \text{atan2}(u_{push, y}, \; u_{push, x}) \cdot \frac{180^\circ}{\pi}$$

---

### 2.2. Punto de Pre-Aproximación ($\vec{P}_{pre}$)
Para empujar el cubo sin golpearlo lateralmente ni desorientarlo, el rover debe primero posicionarse **detrás del cubo**, alineado sobre la recta que une el cubo con el depósito:

$$\vec{P}_{pre} = \vec{P}_{cubo} - d_{pre} \cdot \vec{u}_{push}$$

donde:
* $d_{pre} = 4.0\text{ celdas}$ ($8.0\text{ cm}$). Esta distancia garantiza que el rover pueda alinearse antes del contacto frontal.

```
       [P_pre]  ------>  [P_cubo]  ------------------------->  [P_depot]
     (Rover aquí)         (Cubo)                                (Acopio)
          |                  |                                     |
          +--- d_pre = 8cm --+                                     |
          |                                                        |
          +-------------------- Vector u_push ---------------------+
```

---

## 3. Mitigación de Caída en Bordes (< 5 cm)

Si un cubo se encuentra a menos de $5.0\text{ cm}$ ($2.5\text{ celdas}$) del borde de la cancha, un intento directo de aproximación en línea recta podría colocar al rover fuera de la mesa o forzarlo a maniobrar de espaldas al vacío.

### 3.1. Detección de Borde Crítico
Un cubo está en zona de riesgo perimetral si:

$$\text{Riesgo} = (x_c < 2.5) \lor (x_c > 47.5) \lor (y_c < 2.5) \lor (y_c > 37.5)$$

### 3.2. Generación de Waypoint Diagonal Seguro ($\vec{P}_{safe}$)
En caso de riesgo, en lugar de navegar directo a $\vec{P}_{pre}$, se calcula un punto intermedio en el interior de la cancha:

$$\vec{n}_{borde} = (\Delta x_{interior}, \; \Delta y_{interior})$$

donde:
* Si $x_c < 2.5 \implies \Delta x_{interior} = +4.0\text{ celdas}$
* Si $x_c > 47.5 \implies \Delta x_{interior} = -4.0\text{ celdas}$
* Si $y_c < 2.5 \implies \Delta y_{interior} = +4.0\text{ celdas}$
* Si $y_c > 37.5 \implies \Delta y_{interior} = -4.0\text{ celdas}$

$$\vec{P}_{safe} = \vec{P}_{cubo} + \vec{n}_{borde}$$

El rover navega primero hacia $\vec{P}_{safe}$ y desde allí ejecuta una aproximación angular hacia $\vec{P}_{pre}$, manteniendo su centro de gravedad siempre dentro del área segura.

---

## 4. Control de Guiado y Rumbo Diferencial

### 4.1. Cálculo del Error Angular ($\Delta\theta$)
Dado el rumbo actual del rover $\theta_r$ y el rumbo deseado $\theta_d = \text{atan2}(y_{target} - y_r, \; x_{target} - x_r) \cdot \frac{180^\circ}{\pi}$, el error angular con envoltura en $[-180^\circ, 180^\circ]$ se calcula con:

$$\Delta\theta = \text{atan2}\left(\sin(\theta_d - \theta_r), \; \cos(\theta_d - \theta_r)\right) \cdot \frac{180^\circ}{\pi}$$

### 4.2. Ley de Control de Motores (P-Controller)
Para un accionamiento diferencial con potencias de motor en el rango $[-1.0, 1.0]$:

$$v_{turn} = K_p \cdot \frac{\Delta\theta}{180.0}$$

1. **Giro en su propio eje** (si $|\Delta\theta| > 30^\circ$):
   $$v_{left} = -v_{turn\_max} \cdot \text{sgn}(\Delta\theta), \quad v_{right} = +v_{turn\_max} \cdot \text{sgn}(\Delta\theta)$$
2. **Avance con corrección** (si $|\Delta\theta| \le 30^\circ$):
   $$v_{left} = v_{base} - v_{turn}, \quad v_{right} = v_{base} + v_{turn}$$

Con saturación estricta: $v_{left}, v_{right} \in [-1.0, 1.0]$.

---

## 5. Criterio Oficial de Entrega de Cubo (`cubo_en_su_zona`)

De acuerdo con el algoritmo oficial especificado en [`CONTRATO.md`](file:///home/mamalona/Projects/Church-Boyz/vision-rover/ch-challenge-vision-rover/vision-system/contrato/CONTRATO.md):

Un cubo se considera válidamente depositado si la distancia euclidiana entre el centro del cubo y el centro del depósito es menor o igual al radio efectivo de tolerancia:

$$d_{cd} = \sqrt{(x_c - x_d)^2 + (y_c - y_d)^2}$$

$$d_{cd} \le R_{depot} - \frac{s_{cubo}}{2}$$

Donde:
* $R_{depot} = 3.75\text{ celdas}$ ($75\text{ mm}$ de radio para depósitos de $150\text{ mm}$).
* $s_{cubo} / 2 = 1.25\text{ celdas}$ ($25\text{ mm}$).
* Margen de calificación: $d_{cd} \le 2.50\text{ celdas}$ ($5.0\text{ cm}$).

---

## 6. Maniobra de Retirada Segura (*Safe Retreat*)

Una vez que el cubo está dentro del depósito, retroceder en línea recta provocaría que las aletas o el frontal del rover arrastren el cubo hacia afuera al invertir la marcha.

### 6.1. Cinemática de Reversa Curvada
Se aplica una velocidad asimétrica constante en reversa durante un tiempo predefinido $T_{retreat} = 1.2\text{ s}$:

$$v_{left} = -0.40, \quad v_{right} = -0.20$$

Esto genera una velocidad angular de guiñada en retroceso:

$$\omega = \frac{v_{right} - v_{left}}{L}$$

donde $L$ es el ancho de vía del robot. La trayectoria resultante es un arco de círculo que aleja la aleta exterior del cubo antes de que complete el giro, garantizando que el cubo permanezca $100\%$ dentro del depósito oficial.
