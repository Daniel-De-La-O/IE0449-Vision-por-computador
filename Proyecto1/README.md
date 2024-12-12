# IE0449-Vision-por-computador
Mapeo y análisis de la posición y el movimiento de un objeto en movimiento en una escena con una cámara monocular estática en lenguaje de programación C, haciendo uso de la plataforma de programación CodeBlocks, bajo el sistema operativo Ubuntu 14.04 LTS.

Sea una escena constituida por un objeto en movimiento y una cámara monocular estática. Sea `f` la distancia focal de la cámara, donde el sistema de coordenadas de la imagen `[ima](c, r)` está ubicado en la esquina superior izquierda, y las dimensiones de las imágenes capturadas por la cámara son: `Nfu` píxeles de largo por `Nfv` píxeles de alto. Además, de acuerdo al fabricante de la cámara, el número de sensores por línea es `Ncu`, el número de sensores por columna es `Ncv`, la distancia entre sensores a lo largo del eje horizontal es `Du` y la distancia entre sensores a lo largo del eje vertical es `Dv`.

Sean P <sup>[m]</sup><sub>c ,k</sub>  = (Gckmx, Gckmy, Gckmz, Rckmx, Rckmy, Rckmz)<sup>T</sup> los 6 parámetros que describen la pose del sistema de coodenadas local de la cámara `[c](X, Y, Z)` con respecto al sistema de coordenadas del mundo `[m](x, y, z)` en el instante de tiempo `k`.

Sean P<sup>[c]</sup><sub>o ,k</sub> = (GokcX, GokcY, GokcZ, RokcX, RokcY, RokcZ)<sup>T</sup> los 6 parámetros que describen la pose del sistema de coodenadas local de objeto `[o](r, s, t)` con respecto al sistema de coordenadas local de la cámara `[c](X, Y, Z)` en el instante de tiempo `k`.

Sea H<sup>[o]</sup><sub>o ,k</sub>  = (Hokor, Hokos, Hokot)<sup>T</sup> la posición de un punto arbitrario `Ho` sobre la superficie del objeto con respecto al sistema de coordenadas local del objeto `[o](r, s, t)` en el instante de tiempo `k`.

Sean B<sup>[c]</sup><sub>o ,k -> k+1</sub> = (DTo2cX, DTo2cY, DTo2cZ, DRo2cX, DRo2cY, DRo2cZ)<sup>T</sup> los 6 parámetros que describen el movimiento del sistema de coordenadas local del objeto `[o](r, s, t)` con respecto al sistema de coordenadas local de la cámara `[c](X, Y, Z)` entre los instantes de tiempo `k` y `k + 1`.

Adjunto viene un archivo denominado "current_control_parameters.txt", donde el primer grupo de seis parámetros representa P <sup>[m]</sup><sub>c ,k</sub> , el segundo grupo representa P<sup>[c]</sup><sub>o ,k</sub>, el tercer grupo representa  H<sup>[o]</sup><sub>o ,k</sub>  y el cuarto grupo representa B<sup>[c]</sup><sub>o ,k -> k+1</sub>, así como `f`, `Nfu`, `Nfv`, `Ncu`, `Ncv`, `Du` y `Dv` representan los parámetros intrínsicos de la cámara. Las posiciones y traslaciones se dan en centímetros, así como los ángulos de orientación y ángulos de rotación se dan en grados.

El objetivo de este proyecto es determinar:
- La posición tridimensional H<sup>[c]</sup><sub>o ,k</sub> del punto `Ho` con respecto al sistema de coordenadas local de la cámara `[c](X, Y, Z)` en el instante de tiempo `k`.

- La posición bidimensional h<sup>[pc]</sup><sub>o ,k</sub> del punto `Ho` con respecto al sistema de coordenadas local del plano de la cámara `[pc](u, v)` en el instante de tiempo `k`.

- La posición bidimensional h<sup>[ima]</sup><sub>o ,k</sub> del punto `Ho` con respecto al sistema de coordenadas local de la imagen `[ima](c, r)` en el instante de tiempo `k`.

- La posición tridimensional H<sup>[m]</sup><sub>o ,k</sub> del punto `Ho` con respecto al sistema de coordenadas del mundo `[m](x, y, z)` en el instante de tiempo `k`.

- La posición tridimensional H<sup>[c]</sup><sub>o ,k+1</sub>  del punto `Ho` con respecto al sistema de coordenadas local de la cámara `[c](X, Y, Z)` en el instante de tiempo `k+1`.

- La posición bidimensional h<sup>[pc]</sup><sub>o ,k+1</sub> del punto `Ho` con respecto al sistema de coordenadas local del plano de la cámara `[pc](u, v)` en el instante de tiempo `k+1`.

- La posición bidimensional h<sup>[ima]</sup><sub>o ,k+1</sub> del punto `Ho` con respecto al sistema de coordenadas local de la imagen `[ima](c, r)` en el instante de tiempo `k+1`.

- Los seis parámetros P<sup>[c]</sup><sub>o ,k+1</sub> que describen la pose del objeto con respecto al sistema de coordenadas de la cámara `[c]` en el instante de tiempo `k+1`.

- El vector bidimensional d<sup>[pc]</sup><sub>o ,k -> k+1</sub> que describe el desplazamiento (o flujo óptico) del punto `Ho` sobre el plano de la cámara, entre los instantes de tiempo `k` y `k+1`, respectivamente, con respecto al sistema de coordenadas local del plano de la cámara `[pc](u, v)`.

- El vector bidimensional d<sup>[ima]</sup><sub>o ,k -> k+1</sub>  que describe el desplazamiento (o flujo óptico) del punto `Ho` sobre el plano de la cámara (en píxeles), entre los instantes de tiempo `k` y `k+1`, respectivamente, con respecto al sistema de coordenadas de la imagen `[ima](c, r)`.

Los resultados son desplegados en el terminal y salvados en un archivo de texto denominado "resultados.txt".
