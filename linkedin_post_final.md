# Post 1 — el derrape

Listo para copiar. LinkedIn no renderiza markdown, así que aquí no hay `**`
ni `#`. Los saltos de línea dobles sí funcionan.

Longitudes: post 1.287 caracteres, comentario 1.640.

---

## EL POST

```
Mi robot conseguía el 26% del giro que se le pedía.

La velocidad en línea recta era perfecta. Solo fallaba al girar.

Esa asimetría era el diagnóstico entero.

Es un skid-steer: cuatro ruedas fijas, sin dirección. Gira comandando
distinta velocidad a cada lado, lo que obliga a las ruedas a deslizar
lateralmente contra el suelo. Derrapa por diseño.

Pero la cinemática que traduce "gira a 0,5 rad/s" en velocidades de rueda
asume rodadura pura, sin deslizamiento. Esa hipótesis es exacta yendo recto
— por eso la velocidad lineal iba bien — y es falsa en cuanto giras.

Lo que más me sorprendió fue dónde se manifestaba el fallo.

El controlador local simula 2000 trayectorias por ciclo para decidir cómo
moverse, y las simula asumiendo que el robot consigue lo que se le pide. Con
un cuarto de la autoridad de giro real, esas 2000 predicciones describían un
robot que no existe. Planificaba curvas que físicamente no podía tomar.

Un fallo en la capa más baja, la actuación, se manifestaba como decisiones
absurdas dos capas más arriba.

La primera solución fue una corrección fija en la geometría. Funcionó en un
punto de operación y falló en todos los demás: el derrape no es constante,
depende de la velocidad de giro y del rozamiento. Que una ganancia fija no
bastara es la prueba de que no era un error de geometría.

Al final: un lazo PI cerrado sobre la velocidad angular real.

Robot skid-steer explorando un laberinto sin mapa previo. RTAB-Map para el
SLAM, Nav2 para navegar, explore_lite modificado para decidir a dónde ir.
Isaac Sim y ROS 2 Humble.

Vídeo abajo. Detalle técnico en el primer comentario 👇

#Robotics #ROS2 #ControlSystems #IsaacSim #MobileRobotics
```

---

## EL PRIMER COMENTARIO

Publicarlo tú mismo nada más subir el post.

```
Detalle técnico, por si alguien tiene curiosidad:

MEDIDO
wz pedida: -0,168 rad/s
wz real:   -0,043 rad/s
En ese instante, el 26%.

EL LAZO
PI, no PID. Sin término derivativo a propósito: la velocidad angular medida
es ruidosa y una D solo amplificaría el ruido.

Tres decisiones que importan más que las ganancias:

1. La salida es wz_desired + corrección. La orden original pasa directa y el
lazo solo añade encima. Prealimentación más realimentación: responde bien
desde el primer instante en vez de tener que acumular error para arrancar.

2. Integrador acotado (anti-windup). Si el robot se bloquea contra una pared
el error se mantiene, y sin tope el integrador crecería sin control y daría
un latigazo al liberarse.

3. Banda muerta sobre el comando, no sobre el error. Si se pide una velocidad
angular por debajo del umbral, la salida es cero exacto y el integrador se
reinicia. Evita corregir permanentemente cuando el robot debería ir recto.

El integrador hace el trabajo pesado, porque el derrape es un déficit
sistemático sostenido, no ruido.

POR QUÉ NO BASTABA LO FÁCIL
El primer intento fue agrandar la separación entre ruedas por encima de su
valor físico real, para que el controlador mandase más diferencia entre
lados. Es una ganancia estática: calibras un punto de operación y fallas en
el resto.

UNA NOTA HONESTA
La realimentación sale de la pose real del simulador. En hardware real
tendrías encoders e IMU, y el mismo derrape que intentas corregir
contaminaría la medida. Ese es el salto no trivial a un robot físico.
```

---

## NOTAS DE PUBLICACIÓN

**Los primeros 200 caracteres** son lo único que se ve antes del "ver más".
Aquí son las tres primeras frases, y ya contienen la cifra y la asimetría. Si
alguien no pulsa, al menos se lleva eso.

**Vídeo nativo**, subido a LinkedIn, no enlace a YouTube. El alcance de un
enlace externo cae mucho. Formato cuadrado o vertical; el 16:9 se ve diminuto
en el feed.

**Sin enlaces en el post.** Si quieres poner el de YouTube, va en el comentario
o editado después, por lo mismo.

**El comentario, inmediatamente** después de publicar. Si tarda, la
conversación ya ha arrancado sin él.

---

## LOS OTROS POSTS QUE SALEN DE AQUÍ

No metas esto en el primero. Cada uno aguanta solo.

**2 — El parámetro fantasma.** `orientation_scale` estaba declarado, se leía
del YAML, y solo se usaba para orientar al robot AL LLEGAR. Nunca en la
decisión de a dónde ir. Se podía poner a 100 sin ningún efecto. Y volvió a
pasar con dos parámetros de Nav2 mal anidados que se ignoraban en silencio.
Gancho: "Pasé horas tuneando un parámetro que no hacía nada."

**3 — La distancia en línea recta.** El paquete medía la distancia a cada
frontera con Pitágoras, atravesando muros. Es una simplificación razonable en
espacios abiertos y por eso viene así de serie. Un laberinto es el caso
adversario perfecto. La solución no costó cómputo: la búsqueda que ya
encontraba las fronteras respeta los muros, solo había que ir acumulando.
Gancho: "Una aproximación razonable puede ser justo lo que rompe tu caso."

**4 — La regla que no es un peso.** "No des marcha atrás salvo que no haya
alternativa" no se puede expresar subiendo una ganancia: siempre habrá una
frontera lo bastante grande o cercana como para compensarla. Hubo que cambiar
la forma del término — cobrar el giro relativo al mejor giro disponible, no
en absoluto. En un callejón sin salida el diferencial se anula y el robot
retrocede sin dramas; en un pasillo con salida, lo de atrás paga entero.
Gancho: "Hay reglas que ninguna ganancia puede expresar."

El 4 es el más interesante técnicamente y el que mejor se ve en vídeo: el
plano del cruce donde sigue recto, seguido del callejón donde sí retrocede.
```
