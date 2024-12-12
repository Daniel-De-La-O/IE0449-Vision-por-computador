# IE0449-Vision-por-computador
Procesamiento, análisis y segmentación de imágenes con detección de bordes en tiempo real mediante OpenCV y cámara USB en lenguaje de programación C. 

El objetivo de este proyecto es determinar:
- Leer una imagen de color en formato ".bmp" del disco duro.

- Calcular la imagen de intensidad de la imagen de color obtenida y almacenar la imagen de intensidad resultante en el disco duro bajo el nombre “imagenDeIntensidad.bmp”, en formato “.bmp”.

- Calcular la imagen de gradientes a lo largo del eje X según Sobel de la imagen de intensidad calculada en el punto anterior y almacenar una versión tipo “unsigned char” de la misma en el disco duro bajo el nombre “imagenDeGradientesX.bmp”, en formato “.bmp”.

- Calcular la imagen de gradientes a lo largo del eje Y según Sobel de la imagen de intensidad calculada en el segundo punto y almacenar una versión tipo “unsigned char” de la misma en el disco duro bajo el nombre “imagenDeGradientesY.bmp”, en formato “.bmp”.

- Calcular la imagen de magnitud de gradientes de la imagen de intensidad calculada en el segundo punto a partir de los resultados obtenidos en los dos puntos anteriores, y almacenar una versión tipo “unsigned char” de la misma en el disco duro bajo el nombre “imagenDeMagDeGradientes.bmp”, en formato “.bmp”.

- Umbralizar la imagen de magnitud de gradientes que resultó del punto anterior, con el fin de obtener la imagen de bordes correspondiente, la cual también deberá almacenarse en el disco bajo el nombre “imagenDeBordes.bmp”, en formato “.bmp”. El umbral se introduce manualmente, a través de un archivo de texto como se explicará a continuación.

- Calcular la cantidad de píxeles que fueron clasificados como píxeles de borde en la imagen de bordes calculada en el punto anterior. La cantidad de píxeles se muestra en el terminal y también almacenarse en un archivo de texto denominado “cantidadDePixelesDeBorde.txt”.

- Calcular la media y la varianza de los valores de intensidad de los píxeles que fueron clasificados como píxles de borde en el punto anterior. Los valores de media y varianza se muestran en el terminal y también almacenadas en un archivo de texto denominado “mediaYvarianzaDeIntensidadEnPixelesDeBorde.txt”.

Es importante tomar en cuenta que la imagen de gradientes a lo largo del eje X según Sobel, la imagen de gradientes a lo largo del eje Y según Sobel y la imagen de magnitud de gradientes son imágenes tipo “double”, debido a que la operación en cada pixel da como resultado un valor en coma flotante. Debido a que las funciones para almacenar una imagen en el disco duro requieren que la imagen sea del tipo “unsigned char”, obliga a que cuando se requiera almacenar una imagen tipo “double”, se deba primero generar una versión de la misma tipo “unsigned char”, que posteriormente se le pueda pasar a dichas funciones. Para generar una imagen tipo “unsigned char” de una tipo “double”, se debe recorrer cada píxiel de la imagen tipo “double” y por cada píxel aplicar la siguiente regla: sea W el valor en la posición (x,y) de la imagen tipo “double”, si el valor absoluto de W (fabs(W)) es mayor que 255.0, asignar 255 en la posición (x,y) de la imagen “unsigned char”, en caso contrario asignar el “casting” a “unsigned char” del valor absoluto de W, esto es “(unsigned char) fabs(X)”.
La imagen de entrada se lee de un directorio llamado “input”. Las dimensiones de la imagen (ancho y alto) y el umbral necesario en el antepenúltimo punto, se leen de un archivo de texto de parámetros de control denominado “current_control_parameters.txt”. Las imágenes “imagenDeIntensidad.bmp, “imagenDeGradientesX.bmp”, “imagenDeGradientesY.bmp”, “imageDeMagDeGradientes.bmp” e “imagenDeBordes.bmp”, se almacenan, junto con el archivo de texto “cantidadDePixelesDeBorde.txt” en una carpeta de salida llamada “output”.