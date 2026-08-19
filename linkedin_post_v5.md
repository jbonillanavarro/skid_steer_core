# Post general v5 — registro serio

Post **~1.750 caracteres** · Primer comentario ~1.400

Cambio de registro: nombrar capacidades, no describir comportamientos.

---

## EL POST

```
Robot de exploración autónoma en entornos desconocidos sin GPS.

Entra en un espacio que no ha explorado nunca, lo mapea entero y a partir de ahí
navega a cualquier punto del mapa por la ruta óptima.

Sin mapa previo. Sin waypoints. Sin infraestructura externa de
posicionamiento.

Tres stacks de ros que integrar, un simulador al que construirle el puente a ROS 2,
y un laberinto que rompe varias de las suposiciones que esos paquetes traen
de serie.

EL SIMULADOR — Isaac Sim

Escena propia sobre un modelo de laberinto de rodrivgm (Sketchfab). El
trabajo de verdad fue el puente con ROS 2: cinco action graphs.

/clock, cámara RGB-D (/rgb, /depth, /camera_info), lidar RTX, /odom y /tf, y
la cadena de actuación completa — de /cmd_vel al Differential Controller, y
de ahí a las cuatro juntas vía Articulation Controller.

LA PLATAFORMA — Jackal, skid-steer de cuatro ruedas

RTAB-Map para SLAM RGB-D con cierre de bucle visual. La localización sale
íntegramente de los sensores de a bordo.

Nav2 en dos capas. 
- El planner: NavFn propaga un campo de potencial sobre el costmap a
5 cm por celda y desciende por el gradiente hasta la frontera objetivo. 
El controller: MPPI, cada 50 ms, muestrea 2000 trayectorias a 2,8 segundos vista, las puntúa con 8
críticos y promedia — no sigue la ruta global, la usa como referencia.

explore_lite elige la siguiente frontera, el goal, con una función de coste.

ALGUNOS DE LOS PROBLEMAS SOLUCIONADOS

→ Esa función puntúa fronteras por distancia euclídea y tamaño. Euclídea
significa atravesar muros: en una oficina diáfana es una simplificación
razonable, en un laberinto es justo el caso que la rompe. La reescribí
entera.

→ Un skid-steer gira derrapando, porque sus ruedas no se orientan. La
cinemática diferencial asume rodadura pura, así que la plataforma conseguía
una fracción del giro comandado. Lazo PI cerrado sobre la velocidad angular
real.

En un sistema por capas, cada una asume cosas de la de abajo. Cuando una
suposición falla, el síntoma aparece arriba, lejos de la causa.

ROS 2 Humble · Isaac Sim · 

#Robotics #ROS2 #SLAM #Nav2 #IsaacSim #AutonomousSystems #GPSDenied
#MobileRobotics
```

---

## EL PRIMER COMENTARIO

Publicarlo tú mismo nada más subir el post.

```
Los cinco action graphs, por si alguien está montando algo parecido:

RELOJ — Isaac Read Simulation Time → ROS2 Publish Clock. Parece el más tonto
y es del que cuelga todo: con use_sim_time en true, si el simulador no
publica /clock cada nodo se queda esperando en silencio. Sin errores, sin
avisos. Simplemente nada se mueve.

CÁMARA — Isaac Create Render Product alimentando Camera Helper para color y
profundidad, más Camera Info Helper. Encadenado a Run One Simulation Frame
para que el render vaya sincronizado con la física.

LIDAR — dos RTX Lidar Helpers, cada uno con su render product.

ODOMETRÍA Y TF — Compute Odometry publicando /odom, más los árboles de
transformadas.

ACTUACIÓN — el más largo. Subscribe Twist recibe /cmd_vel, un Script Node en
Python aplica la corrección de velocidad angular, el Differential Controller
convierte (v, ω) en velocidades de rueda y el Articulation Controller las
reparte a las cuatro juntas.

Un detalle del que se habla poco: la odometría sale de la pose real del
chasis, no de integrar velocidades de rueda. Por eso el derrape nunca
contaminó el mapa — solo se manifestaba como "gira menos de lo que le pido".
En un robot físico, con encoders, ese mismo derrape ensuciaría también la
localización. Ahí está el salto no trivial a hardware real.
```

---

## LOS CAMBIOS Y POR QUÉ

**"Robot de exploración autónoma" recuperado.** Tenías razón y cambiarlo fue un
error mío. Nombrar la categoría dice que conoces el campo; describir el
comportamiento suena a estar explicándolo por primera vez. En un sector donde
las capacidades tienen nombre propio, usar el nombre no es cosmética.

**"sin GPS" y "sin infraestructura externa de posicionamiento".** Esto es lo
que más te va a cambiar quién lee el post.

Lo que has construido tiene nombre en defensa: **navegación en entornos
GPS-denied**. Es un área de capacidad reconocida, y tu sistema encaja
literalmente — sin posicionamiento externo, localización íntegra desde
sensores de a bordo, sin mapa previo. Un interior sin GPS *es* el problema.

No infla nada. Solo lo dice en el vocabulario de quien buscas que lo lea, y
añade una etiqueta (#GPSDenied) por la que esa gente busca de verdad.

**Añadida la frase "la localización sale íntegramente de los sensores de a
bordo".** Refuerza lo anterior con un hecho técnico verificable, no con un
adjetivo.

**"EL ROBOT" → "LA PLATAFORMA".** Es el término del sector. Lo mismo con
"la plataforma conseguía una fracción del giro" más abajo.

**Estructura de la apertura.** Primero la categoría en una línea sola, después
qué hace, después qué no necesita. Tres bloques cortos y declarativos en vez
de una frase larga. Firmeza es economía: frases que afirman y paran.

---

## UNA DECISIÓN QUE TE DEJO A TI

Quité **"Uno de mis proyectos de verano"**.

Es una fórmula que se disculpa por adelantado: sitúa el trabajo como algo
hecho en ratos libres antes de que nadie haya podido juzgarlo. Contra el
objetivo de "firme y serio", juega en contra.

Si lo que buscas es no dar a entender que fue un encargo profesional, hay
formas de decirlo sin restarle peso — al final del post, no al principio:

> Proyecto personal. Código y detalle técnico en el repositorio.

Ahí ya no condiciona la lectura: llega cuando el lector ya ha visto el
sistema. Si prefieres mantenerlo arriba, es tu decisión, pero conviene saber
lo que cuesta.

---

## LA SERIE

| | Tema | Gancho |
|---|---|---|
| 1 | **Este** — el sistema completo | Exploración autónoma sin GPS |
| 2 | El derrape del skid-steer | Conseguía el 26% del giro comandado |
| 3 | El parámetro que no hacía nada | Horas ajustando un parámetro fantasma |
| 4 | La regla que no es un peso | Hay reglas que ninguna ganancia expresa |

Para el objetivo que me dices, el 4 es el que más te conviene publicar
segundo. Trata de cómo se codifica una regla de comportamiento —"no
retrocedas salvo que no haya alternativa"— cuando no se puede expresar con un
peso. Eso es diseño de autonomía, no ajuste de parámetros, y es exactamente
la conversación que interesa en ese sector.
