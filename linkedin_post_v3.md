# Post general v3 — el sistema completo

Recorte y afinado sobre tus apuntes.

Post **~1.750 caracteres** (antes 2.400) · Primer comentario ~1.400

---

## EL POST

```
Uno de mis proyectos de verano: un robot de exploración en un entorno que no ha
visto nunca, lo mapea entero y es capaz de ir a cualquier zona mapaeada siguiendo la ruta mas optima.

Sin mapa previo. Sin waypoints. Sin nadie al mando.

Lo difícil no fue instalar los paquetes. Fue entender qué asume cada uno y
encontrar dónde esas suposiciones no valían para mi entorno.

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

explore_lite elige la siguiente frontera (el goal) en base a una funcion de costes.

LO QUE NO VENÍA HECHO

→ explore_lite puntúa fronteras por distancia euclídea y tamaño. Euclídea
significa atravesar muros: en una oficina diáfana es una simplificación
razonable, en un laberinto es justo el caso que la rompe. Reescribí la
función de coste.

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

## QUÉ HE CAMBIADO SOBRE TU VERSIÓN

**Recortado de 2.400 a ~1.750 caracteres.** Lo que salió: el párrafo de qué
parámetros tocar (se solapaba con la frase anterior), la explicación larga de
explore_lite al presentarlo (ya se explica en "lo que no venía hecho"), y el
punto del planificador por territorio desconocido — es el más interesante de
los tres, pero también el que más contexto necesita, y sin él el post gana en
ritmo. Lo guardé para el post 4.

**Nombres reales en vez de descripciones.** `Force3DoF`, "campo de potencial",
"desciende por el gradiente", "8 críticos" (el término de Nav2, no
"criterios"), `denoise_layer`, `Differential Controller`, `Articulation
Controller`. Quien sabe lo reconoce; quien no, entiende la frase igual.

**"EL ROBOT — Jackal, skid-steer de cuatro ruedas".** Como pedías. Y además
prepara el segundo guion: cuando dos párrafos después dices que gira
derrapando, ya sabemos por qué.

**`Force3DoF` sin interrogante.** Está en tu launch junto con
`Optimizer/Slam2D`, es correcto.

**El ruido de los costmaps entra como tercer punto.** Tenías razón en que era
interesante, y encaja mejor que el del planificador porque se resume en una
línea.

**Arreglado lo de NavFn.** Tu versión decía "cada cuadrito de 0.05 m calcula la
ruta óptima hasta la frontera". Lo que hace es propagar un campo de potencial
desde la meta —el coste acumulado de llegar desde cada celda— y luego
descender por el gradiente. Decirlo así es más corto y más preciso.

**Eliminado el "un mes".** Tu "proyecto de verano" funciona mejor: dice lo
mismo sin sonar a queja.

**El apunte del reloj se movió al comentario.** Es un detalle excelente, pero
en el post ocupaba cuatro líneas de las que quieres recortar. En el comentario
respira mejor y sigue siendo lo primero que lee quien baja.

---

## SI QUIERES BAJAR A ~1.200

Se puede recortar más, pero ya cuesta:

- Fundir las tres viñetas de "lo que no venía hecho" en una sola frase cada
  una, sin el "en una oficina diáfana / en un laberinto"
- Quitar la línea de explore_lite al presentarlo, ya que reaparece abajo
- Quitar el cierre de "en un sistema por capas"

Yo no lo haría. Ese cierre es lo único del post que no habla de tecnología, y
es lo que puede compartir alguien que no trabaja con ROS.

---

## LA SERIE

| | Tema | Gancho |
|---|---|---|
| 1 | **Este** — el sistema completo | Proyecto de verano |
| 2 | El derrape del skid-steer | Conseguía el 26% del giro que pedía |
| 3 | El parámetro que no hacía nada | Horas tuneando un parámetro fantasma |
| 4 | La regla que no es un peso | Hay reglas que ninguna ganancia expresa |

El 4 lleva ahora también el punto del planificador por territorio desconocido:
"que se equivoque no es el fallo, el fallo es qué pasa cuando se equivoca".
Es el más interesante técnicamente y el que mejor se ve en vídeo — el cruce
donde el robot sigue recto pudiendo girar, seguido del callejón sin salida
donde sí retrocede.
