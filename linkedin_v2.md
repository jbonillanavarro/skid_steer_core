# Exploración autónoma de un laberinto — notas para LinkedIn / YouTube

Versión revisada de `linkedin.txt`. Mismo contenido, reordenado para que se
entienda sin conocer el proyecto, con las cifras verificadas contra el código
y la configuración reales.

Criterio: **solo entra lo comprobado.** Al final hay una sección aparte con lo
que observamos pero no llegamos a confirmar, para no mezclarlo con el resto.

---

## 1. Qué es

Un robot skid-steer explora un laberinto que no conoce, lo mapea entero y
decide él solo a dónde ir después. Sin mapa previo, sin waypoints, sin nadie
al mando.

Simulado en Isaac Sim, sobre ROS 2 Humble.

---

## 2. Los tres nodos

### SLAM — RTAB-Map

Construye el mapa mientras se mueve, con cámara RGB-D. Genera dos cosas: una
rejilla de ocupación 2D para navegar, y el grafo 3D con cierres de bucle.

Configurado con `Reg/Force3DoF` y `Optimizer/Slam2D`: el robot vive en un
plano, y forzarlo elimina la deriva en cabeceo y balanceo, que en interiores
no aporta nada y sí mete ruido.

Un dato que conviene tener presente: `Grid/RangeMax: 4.0`. La rejilla solo
marca hasta 4 metros. Es el alcance real de todo el sistema — cualquier
parámetro de rango mayor aguas abajo, en Nav2, no puede ver más lejos de lo
que esta capa le da.

### Navegación — Nav2, en dos capas

**Planificador global: NavFn con Dijkstra.** Calcula la ruta completa hasta la
meta sobre el costmap global.

Funciona en dos fases. Primero propaga una onda desde la meta y asigna a cada
celda un *potencial*: el coste acumulado de llegar desde ahí hasta el
destino. Después se coloca en la posición del robot y desciende siempre hacia
el vecino de menor potencial. Como ese campo no tiene mínimos locales, el
descenso nunca se atasca.

Detalle importante para exploración: `allow_unknown: true`. La meta está por
definición en territorio sin mapear, así que sin esta opción el planificador
rechazaría todas las metas y el robot no saldría nunca.

Y algo que suele sorprender: **NavFn no planifica ningún giro.** Trabaja sobre
una rejilla 2D y devuelve puntos XY. No tiene concepto de orientación.

**Controlador local: MPPI** (Model Predictive Path Integral, una variante de
MPC basada en muestreo). Es quien manda las velocidades al robot.

Cada ciclo, 20 veces por segundo:

1. Parte de la secuencia de control del ciclo anterior y genera **2000
   variaciones** metiéndole ruido a (v, ω)
2. **Simula** cada una con cinemática directa, 56 pasos de 50 ms → 2.8 s de
   horizonte
3. **Puntúa** cada trayectoria resultante con 8 críticos
4. Convierte coste en peso y hace la **media ponderada de las secuencias de
   control**
5. Publica el primer comando de esa media en `/cmd_vel`

Dos cosas que se malinterpretan siempre:

- **No elige la mejor de las 2000.** Las promedia todas, ponderadas. La
  trayectoria que ejecuta no estaba en el conjunto muestreado: emerge de él.
- **Las trayectorias solo sirven para puntuar.** El promedio se hace sobre las
  secuencias de comandos, no sobre los recorridos. Se simula para poder
  evaluar, y luego se tira la simulación.

La ruta global no es una lista de destinos, es **una referencia para
puntuar**. Por eso el robot se separa de ella en las curvas: no está fallando
al seguir un punto, está decidiendo que apartarse puntúa mejor.

### Exploración — explore_lite (modificado)

Detecta *fronteras* —los límites entre lo mapeado y lo desconocido— y decide a
cuál ir. Sección siguiente.

---

## 3. La función de costes

La original del paquete solo tenía dos términos: distancia euclídea y tamaño.
Los otros dos son añadidos, y responden a problemas concretos que dio el
laberinto.

```
coste = 0.05 · (25 · d_BFS − 2 · celdas)      ← distancia y tamaño
      + 20.0 · (θ² − θ²_mín)                  ← giro
      −  2.5 · clamp(p + 4.0, 0, 8)           ← compromiso
```

Se elige la frontera de **menor coste**.

### La regla para leer cualquier término

Con estos valores, **1.25 unidades de coste = 1 metro de conducción**. Divide
cualquier término entre 1.25 y sabes cuántos metros de rodeo estás dispuesto a
pagar por él. Es la única forma de saber si un parámetro está bien puesto.

### Distancia — `potential_scale: 25.0`

`d_BFS` es la distancia por el **camino real**, recorriendo celdas
transitables desde el robot, no la línea recta.

Ese recorrido no hubo que inventarlo: el paquete ya hacía una búsqueda en
anchura sobre el mapa para *encontrar* las fronteras, y esa búsqueda respeta
los muros por construcción. Solo había que ir acumulando la distancia en el
mismo barrido. Coste computacional añadido: cero.

Es una aproximación 4-conectada (sin diagonales), así que en tramos diagonales
sobrestima. Más fiel que la línea recta, no idéntica al plan real de NavFn.

### Tamaño — `gain_scale: 2.0`

Número de celdas frontera del grupo. Resta, o sea premia: una frontera grande
destapa más mapa por metro recorrido.

*(Sobre el 0.05: en el término de tamaño convierte celdas a metros de borde.
En el de distancia es solo un factor de escala heredado del código original,
porque `d_BFS` ya viene en metros. Coeficientes efectivos: 1.25 por metro de
distancia, 0.1 por celda de frontera.)*

### Giro — `orientation_scale: 20.0`

`θ` es el ángulo entre el rumbo actual del robot y el rumbo de la **ruta BFS
real**, no la dirección en línea recta al centroide. Una frontera al otro lado
de un muro puede parecer "de frente" y exigir dar la vuelta entera; con el
rumbo de la ruta real, eso se cobra.

Al cuadrado, para que los giros pequeños salgan casi gratis y los de casi 180°
se disparen. Una escala lineal no puede dar esa forma.

**Y es relativo, no absoluto.** Se resta `θ²_mín`, el mejor giro disponible en
ese ciclo. Ahí está la idea central:

- **Callejón sin salida**, donde todas las fronteras exigen media vuelta: el
  diferencial se anula, nadie paga, y deciden distancia y tamaño. El robot
  retrocede sin dramas.
- **Pasillo con una frontera de frente**: la de atrás paga el diferencial
  entero.

Eso es exactamente *"no des marcha atrás salvo que no haya alternativa"*, que
es lo que se quería y que **ningún peso escalar puede expresar** — siempre
habrá una frontera lo bastante grande o cercana como para compensarlo. Hubo
que cambiar la forma del término, no su magnitud.

Ventaja secundaria: restar el mínimo desplaza todos los costes por igual, así
que el orden entre fronteras no salta cuando la más recta desaparece del mapa.

### Compromiso — `commitment_scale: 2.5`

Solo se aplica a la frontera que ya se está persiguiendo (misma meta dentro de
`goal_identity_tolerance: 0.5 m`).

- `p` = progreso real = distancia al elegirla − distancia actual
- `commitment_hysteresis: 4.0` = ventaja fija desde el instante cero. Es lo
  que rompe el empate cuando todas las opciones están lejos y sus costes
  quedan pegados y fluctuando entre ciclos
- `commitment_max: 8.0` = **techo**. `2.5 × 8 = 20` unidades ≈ 16 m de ventaja
  máxima

El techo es el parámetro crítico, no la escala. Sin él el bono crece sin
límite y la meta deja de ser una preferencia para volverse una condena: el
robot la persigue aunque acabe siendo una pared.

---

## 4. El problema del derrape

El Jackal es un robot skid-steer de cuatro ruedas. Gira comandando distinta
velocidad a cada lado, y como las ruedas no pueden orientarse, se ven
obligadas a **deslizar lateralmente contra el suelo** para completar el giro.
Eso es el derrape.

La consecuencia: la velocidad angular real se quedaba muy por debajo de la
comandada. **La lineal iba bien.**

Y esa asimetría es la clave del diagnóstico. Avanzar en línea recta es
rodadura pura y la cinemática diferencial la describe exactamente. Girar
implica un deslizamiento que esa misma cinemática **asume que no existe**.
Todo lo que se va en el rascado es giro que no ocurre.

Un dato medido, en un instante concreto: `wz_desired = −0.168 rad/s` frente a
`wz_real = −0.043 rad/s`.

### Por qué importa más de lo que parece

MPPI simula 2000 trayectorias por ciclo **asumiendo que el robot consigue lo
que se le pide**. Si no lo consigue, esas 2000 predicciones describen un robot
que no existe: se planifican curvas que físicamente no se pueden tomar.

No es que el controlador eligiera mal entre opciones buenas. Es que estaba
evaluando otro robot. Por eso un fallo en la capa más baja —la actuación— se
manifestaba como decisiones absurdas dos capas más arriba.

### La solución: un PI en Isaac Sim

Un nodo de script en el action graph, con control proporcional-integral, que
compensa la diferencia entre la velocidad angular pedida por MPPI (`/cmd_vel`)
y la real del robot (leída con un nodo *Read Prim Attribute*).

```python
def setup(db):
    db.internal_state.integral = 0.0

def compute(db):
    wz_desired = db.inputs.wz_desired
    wz_real = db.inputs.wz_real
    dt = db.inputs.dt if db.inputs.dt > 0.0 else 1.0 / 60.0
    deadband = db.inputs.deadband

    if abs(wz_desired) < deadband:
        db.internal_state.integral = 0.0
        db.outputs.wz_corrected = 0.0
        return

    error = wz_desired - wz_real

    new_integral = db.internal_state.integral + error * dt
    limit = db.inputs.integral_limit
    db.internal_state.integral = max(-limit, min(limit, new_integral))

    correction = db.inputs.Kp * error + db.inputs.Ki * db.internal_state.integral
    db.outputs.wz_corrected = wz_desired + correction

def cleanup(db):
    db.internal_state.integral = 0.0
```

Cuatro decisiones de diseño que se ven en ese código:

**No hay término derivativo.** Es un PI, no un PID. La velocidad angular
medida es ruidosa y una D solo amplificaría ese ruido.

**La salida es `wz_desired + correction`.** No es un PI puro: la orden original
pasa directa y el lazo solo añade la corrección encima. Es prealimentación más
realimentación, y hace que el sistema se comporte bien desde el primer
instante en vez de tener que acumular error para arrancar.

**El integrador está acotado** (`integral_limit`). Es anti-windup: si el robot
se queda bloqueado contra una pared, el error se mantiene y sin ese tope el
integrador crecería sin control, provocando un latigazo al liberarse.

**La banda muerta se aplica al comando, no al error.** Si se pide una
velocidad angular por debajo del umbral, la salida es exactamente cero y el
integrador se reinicia. Evita que el lazo esté corrigiendo permanentemente
cuando el robot debería ir recto.

El integrador es el que hace el trabajo pesado aquí, porque el derrape es un
déficit sistemático sostenido, no ruido.

### Por qué no bastaba una corrección fija

El primer intento fue agrandar el parámetro de separación entre ruedas por
encima de su valor físico real, para que el controlador mandara más diferencia
entre lados. Funciona en un punto de operación.

Pero **el derrape no es constante**: depende de la velocidad de giro, del
rozamiento y de si el robot gira parado o girando mientras avanza. Una
ganancia fija calibra un caso y falla en el resto. Que esa solución no fuera
suficiente es, en sí misma, la prueba de que no era un simple error de
geometría — un error de geometría sería un factor constante y se arreglaría
con un número.

---

## 5. Las ideas que hacen buen post

Por si hay que elegir qué contar:

**Un parámetro puede estar declarado, leerse del YAML y no hacer nada.** Nos
pasó dos veces. En explore_lite, `orientation_scale` existía pero solo se
usaba para orientar al robot *al llegar*, nunca en la decisión de a dónde ir:
se podía poner a 100 sin ningún efecto. Y en Nav2, dos parámetros de
visualización mal anidados se ignoraban en silencio. En ninguno de los dos
casos hubo un aviso. Es el fallo más caro que existe, porque te pasas horas
buscando el problema en otro sitio.

**Una aproximación razonable puede ser exactamente lo que rompe tu caso.** La
distancia en línea recta es una simplificación sensata en espacios abiertos, y
por eso el paquete viene así. Un laberinto es el caso adversario perfecto:
maximiza la diferencia entre "cerca en el mapa" y "cerca de verdad".

**Los síntomas mienten y las capas se contaminan.** Un fallo de actuación se
manifestaba como decisiones absurdas del planificador. Y la observación
opuesta: encontrar la causa exigió instrumentar y reconstruir la aritmética a
mano desde los logs. Las hipótesis a ojo fallaron todas.

**Hay reglas que no son un peso.** "No retrocedas salvo que no haya
alternativa" no se puede expresar subiendo una ganancia, por alta que sea. Hay
que cambiar la forma del término.

---

## 6. Lo observado pero no confirmado

Aparte, para no mezclarlo con lo anterior. Si alguien pregunta, es honesto
decir que está sin cerrar.

- **El robot se quedaba bloqueado frente a paredes.** Con la ruta visualizada
  se vio que el planificador trazaba camino a través de zonas que en el mapa
  no estaban marcadas como obstáculo. Eso es coherente con `allow_unknown` y
  con los huecos sin mapear, pero no llegamos a caracterizar el mecanismo
  completo.
- **El verificador de progreso de Nav2 no abortaba.** Observado: 17 segundos
  con el robot inmóvil y ningún abort. La hipótesis es que cada ruta nueva
  reinicia el contador, pero no está verificado en el código.
- **Los huecos en el mapa.** Pequeños, de un metro como mucho, y normalmente
  en zona desconocida más que borrada. Causa sin determinar.
- **Saturación de par en las juntas.** Podría contribuir al déficit de giro
  junto al derrape. Se distinguiría comparando el déficit a distintas
  velocidades de giro; no se hizo.
