# Post general v4 — el sistema completo

Post **~1.700 caracteres** · Primer comentario ~1.400

---

## EL POST

```
Uno de mis proyectos de verano: un robot que explora un entorno que no ha
visto nunca, lo mapea entero y después puede ir a cualquier punto del mapa
por la ruta óptima.

Sin mapa previo. Sin waypoints. Sin nadie al mando.

Tres stacks que integrar, un simulador al que construirle el puente a ROS 2,
y un laberinto que rompe varias de las suposiciones que esos paquetes traen
de serie.

EL SIMULADOR — Isaac Sim

Escena propia sobre un modelo de laberinto de rodrivgm (Sketchfab). El
trabajo de verdad fue el puente con ROS 2: cinco action graphs.

/clock, cámara RGB-D (/rgb, /depth, /camera_info), lidar RTX, /odom y /tf, y
la cadena de actuación completa — de /cmd_vel al Differential Controller, y
de ahí a las cuatro juntas vía Articulation Controller.

EL ROBOT — Jackal, skid-steer de cuatro ruedas

RTAB-Map para SLAM RGB-D con cierre de bucle visual.

Nav2 en dos capas. NavFn propaga un campo de potencial sobre el costmap a
5 cm por celda y desciende por el gradiente hasta la frontera objetivo. MPPI,
cada 50 ms, muestrea 2000 trayectorias a 2,8 segundos vista, las puntúa con 8
críticos y promedia — no sigue la ruta global, la usa como referencia.

explore_lite elige la siguiente frontera, el goal, con una función de coste.

LO QUE NO VENÍA HECHO

→ Esa función puntúa fronteras por distancia euclídea y tamaño. Euclídea
significa atravesar muros: en una oficina diáfana es una simplificación
razonable, en un laberinto es justo el caso que la rompe. La reescribí
entera.

→ Un skid-steer gira derrapando, porque sus ruedas no se orientan. La
cinemática diferencial asume rodadura pura, así que el robot conseguía una
fracción del giro comandado. Lazo PI cerrado sobre la velocidad angular real.

→ El ruido de la RGB-D metía obstáculos fantasma en los costmaps. Filtrado en
la rejilla de RTAB-Map y denoise_layer en Nav2.

En un sistema por capas, cada una asume cosas de la de abajo. Cuando una
suposición falla, el síntoma aparece arriba, lejos de la causa.

ROS 2 Humble · Isaac Sim · Vídeo 👇

#Robotics #ROS2 #SLAM #Nav2 #IsaacSim #MobileRobotics #AutonomousSystems
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

## LOS CAMBIOS DE ESTA VERSIÓN

**Fuera "lo difícil no fue instalar los paquetes".** Tenías razón, y el fallo
es de manual: esa frase **nombra justo aquello de lo que quieres distanciarte**.
Negar algo no lo borra, lo planta en la cabeza de quien lee. Nadie se presenta
desmintiendo la versión pobre de su propio trabajo.

En su lugar:

> Tres stacks que integrar, un simulador al que construirle el puente a ROS 2,
> y un laberinto que rompe varias de las suposiciones que esos paquetes traen
> de serie.

Es afirmativa, y encima trabaja: **anuncia las tres secciones que vienen** —el
simulador, el robot, y lo que hubo que arreglar. La anterior no estructuraba
nada.

Además engancha con el cierre. Abres diciendo que el laberinto rompe
suposiciones y cierras con "cuando una suposición falla, el síntoma aparece
arriba". El post queda cerrado sobre sí mismo.

**"la ruta más óptima" → "la ruta óptima".** Óptima ya es el superlativo; "más
óptima" es el tipo de detalle que delata a quien escribe de oído. Justo lo
contrario de lo que buscas.

**Corregido "mapaeada".**

**Reordenada la primera frase.** "Un robot de exploración en un entorno que no
ha visto nunca" mezclaba el qué y el dónde. Ahora: explora → mapea → navega.
Tres verbos en orden, que es como se cuenta un sistema.

**"explore_lite elige la siguiente frontera, el goal, con una función de
coste"** y abajo *"Esa función puntúa..."*. El paréntesis cortaba el ritmo, y
enlazar con "esa" ahorra repetir el nombre.

---

## LA SERIE

| | Tema | Gancho |
|---|---|---|
| 1 | **Este** — el sistema completo | Proyecto de verano |
| 2 | El derrape del skid-steer | Conseguía el 26% del giro que pedía |
| 3 | El parámetro que no hacía nada | Horas tuneando un parámetro fantasma |
| 4 | La regla que no es un peso | Hay reglas que ninguna ganancia expresa |

El 4 lleva también el punto del planificador por territorio desconocido: "que
se equivoque no es el fallo, el fallo es qué pasa cuando se equivoca". Es el
más interesante técnicamente y el que mejor se ve en vídeo — el cruce donde el
robot sigue recto pudiendo girar, seguido del callejón sin salida donde sí
retrocede.
