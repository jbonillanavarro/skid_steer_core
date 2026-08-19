# Post general — el sistema completo

Objetivo: demostrar capacidad para montar un entorno de robótica complejo.
No es un post sobre un bug, es un post sobre un sistema.

Post ~2.400 caracteres · Primer comentario ~1.500

---

## EL POST

```
Uno de mis proyectos de verano: un robot capaz de explorar cualquier entorno que no ha visto
nunca, lo mapea entero y decide él solo a dónde ir después siguiendo el camino mas optimo.

Sin mapa previo. Sin waypoints. Sin nadie al mando.

Lo difícil no fue instalar los paquetes. Fue decidir cuáles, entender qué
asume cada uno, y encontrar dónde esas suposiciones no valían para mi
entorno y que paramtros de configuracion debo de tocar para que mi robot se comporte correctamente.

EL SIMULADOR (Isaac Sim)

Escena en Isaac Sim, con el laberinto a partir de un modelo de rodrivgm
(Sketchfab). Pero el trabajo de verdad fue el puente con ROS 2: cinco action
graphs, uno por función.

Publicador de reloj de simulación (/clock), publicador de cámara RGB-D para el SLAM (/camera_info /depth /rgb), lidar RTX, odometría ()/odom y árbol de transformadas (/tf), y la cadena de actuación completa: cmd_vel entra, se corrige,
pasa por el controlador diferencial y sale repartido a las cuatro ruedas.

EL ROBOT

RTAB-Map — SLAM 3D con cámara RGB-D y cierre de bucle visual. Forzado a 3
grados de libertad?: el robot vive en un plano, y restringirlo elimina una
deriva que en interiores solo mete ruido. ? nota: la solucion que obtuvimos para arreglar el tema del ruido en los costamps puede ser interesante como nota

Nav2, en dos capas con responsabilidades distintas.

El planificador global (NavFn, Dijkstra) calcula la ruta completa. Cada cuadrito de 0.05m calcula la ruta optima hasta la frontera objetivo decidad por el explore lite.

El controlador local (MPPI) decide el movimiento real. Cada 50 ms simula 2000
trayectorias 2,8 segundos hacia delante, las puntúa con 8 criterios y
promedia. No sigue la ruta global: la usa como referencia y decide por su
cuenta.

explore_lite — detecta las fronteras entre lo mapeado y lo desconocido y
elige a cuál ir.

LO QUE NO VENÍA HECHO

→ explore_lite decide a qué frontera ir con dos factores: distancia y tamaño.
Y mide esa distancia euclidea, atravesando muros. En una oficina
diáfana es razonable. Un laberinto es justo el caso que lo rompe. Reescribí
la función de coste entera.

→ El robot es skid-steer: gira derrapando, porque sus ruedas no se orientan.
La cinemática estándar asume que no derrapa, así que conseguía una fracción
del giro que se le pedía. Hizo falta un lazo de control cerrado sobre la
velocidad angular real, un control de acutación.

Lo que me llevo: en un sistema por capas, cada una asume cosas de la de
abajo. Cuando una suposición no se cumple, el síntoma aparece arriba, lejos
de la causa.

ROS 2 Humble · Isaac Sim · Vídeo abajo 👇

#Robotics #ROS2 #SLAM #Nav2 #IsaacSim #MobileRobotics #AutonomousSystems
```

---

## EL PRIMER COMENTARIO

Publicarlo tú mismo nada más subir el post. Aquí va el detalle de Isaac Sim,
que es lo que más preguntas suele generar.

```
Los cinco action graphs, por si alguien está montando algo parecido:

RELOJ — Isaac Read Simulation Time → ROS2 Publish Clock. Es la base de todo
lo demás. Con use_sim_time en true, cada nodo de ROS depende de este topic.

CÁMARA — Isaac Create Render Product alimentando Camera Helper para color y
profundidad, más Camera Info Helper para la calibración. Encadenado a Run One
Simulation Frame para que el render vaya sincronizado con la física.

LIDAR — dos RTX Lidar Helpers, cada uno con su render product.

ODOMETRÍA Y TF — Compute Odometry publicando odom, más los árboles de
transformadas. La odometría sale de la pose real del chasis, no de integrar
velocidades de rueda.

ACTUACIÓN — el más largo. Subscribe Twist recibe cmd_vel, un Script Node en
Python aplica la corrección de velocidad angular, el Differential Controller
convierte velocidad lineal y angular en velocidades de rueda, y un
Articulation Controller las reparte a las cuatro juntas.

Ese detalle de la odometría importa más de lo que parece: como sale de la
física real y no de las ruedas, el derrape nunca contaminó el mapa. Solo se
manifestaba como "gira menos de lo que le pido". En un robot físico, con
encoders, el mismo derrape ensuciaría también la localización — y ese es el
salto no trivial a hardware real.
```

---

## SOBRE LO DE LA CÁMARA

Dijiste que se mostraba girada y que el mapeo salía mal, pero que no recuerdas
el detalle. **Lo he dejado fuera del post a propósito.**

Es un problema clásico —ROS usa para cámaras una convención distinta de la del
resto del sistema, y el simulador otra— pero como no recuerdas cuál fue la
causa exacta ni el arreglo, contarlo a medias es justo lo que puede llevarte a
un comentario incómodo pidiendo detalles.

Si te apetece recuperarlo, se comprueba en un minuto: mira la orientación del
prim de la cámara en la escena y el `frameId` que pusiste en el Camera Helper.
Si aparece una rotación que no es la identidad, ahí está la respuesta. Me
dices y lo añado, porque como punto del post es bueno: cualquiera que haya
integrado una cámara en ROS lo reconoce al instante.

---

## POR QUÉ ESTÁ ESCRITO ASÍ

**Isaac Sim va primero.** Es lo que más te diferencia. Gente que ha usado Nav2
hay mucha; gente que ha construido el puente simulador-ROS desde cero, bastante
menos. Y era lo que peor reflejado estaba en la versión anterior.

**El apunte del reloj es el mejor detalle del post.** Es humilde, concreto y
solo lo sabe quien lo ha sufrido. Ese tipo de detalle convence más que
cualquier lista de tecnologías.

**Cada nodo lleva su porqué, no solo su nombre.** "Dijkstra y no A* porque el
mapa cambia" demuestra criterio; "usé NavFn" no demuestra nada.

**Acreditar el modelo del laberinto.** Nadie espera que modeles un laberinto, y
ser preciso en lo pequeño hace creíble lo grande.

**Sin cifras de depuración.** Nada de "26%" ni "1240 unidades de coste". Eso es
para los monográficos; aquí distraería del mensaje, que es la amplitud.

**Los primeros 200 caracteres** —lo único visible antes del "ver más"— son las
dos primeras frases: qué hace el sistema y cuánto costó.

---

## LA SERIE

Este va primero: presenta el sistema. Los demás profundizan, y cada uno
funciona solo porque ya existe el contexto.

| | Tema | Gancho |
|---|---|---|
| 1 | **Este** — el sistema completo | Un mes montando esto |
| 2 | El derrape del skid-steer | Conseguía el 26% del giro que pedía |
| 3 | El parámetro que no hacía nada | Horas tuneando un parámetro fantasma |
| 4 | La regla que no es un peso | Hay reglas que ninguna ganancia expresa |

El 4 es el más interesante técnicamente y el que mejor se ve en vídeo: el
cruce donde el robot sigue recto pudiendo girar, seguido del callejón sin
salida donde sí retrocede. Los dos planos juntos explican la idea sin una sola
palabra.
