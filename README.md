[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-f059dc9a6f8d3a56e377f745f24479a46679e63a5d9fe6f495e02850cd0d8118.svg)](https://classroom.github.com/online_ide?assignment_repo_id=7421157&assignment_repo_type=AssignmentRepo)
# RoboCup

## Creators:
Marina Wiesenberg Bustillo <br />
Juan Miguel Valverde García <br />
Daniel Quinga López <br />
David Duro Aragonés <br />

## 0. Índice
   - 0.0 ¿Qué pruebas elegimos y por qué?
   - 1. Prueba 1
      - 1.0 Explicación de código  
      - 1.1 ¿Cuál fue nuestra idea principal?
      - 1.2 ¿Cómo tuvimos que modificarlo?
      - 1.3 Relación entre plan y ejecución
   - 2. Prueba 2
      - 2.0 Explicación de código  
      - 2.1 ¿Cuál fue nuestra idea principal?
      - 2.2 ¿Cómo tuvimos que modificarlo?
      - 2.3 Relación entre plan y ejecución
   - 3. Prueba 3
      - 3.0 Explicación de código  
      - 3.1 ¿Cuál fue nuestra idea principal?
      - 3.2 ¿Cómo tuvimos que modificarlo?
      - 3.3 Relación entre plan y ejecución
   - 4. Problemas Generales
   

## 0.0 ¿Qué pruebas elegimos y por qué?
Al comenzar a planear el funcionamiento del kobuki en cada una de nuestras pruebas pensamos y decidimos que las más sencillas serían la 1ª y la 2ª, ya que creando un buen programa base de navegación tendríamos todo casi hecho.

## 1. Prueba 1

![Carry my luggage BT](https://user-images.githubusercontent.com/90764439/167626045-eca06e1c-5ec1-4d95-b531-305f9fb84226.png)
## 1.0 Explicación de código

Primero se espera la orden de start, después, el robot debe navegar hasta el árbitro, de forma que con puntos previamente apuntados, se crea una ruta del robot al punto, al usar navegación en un entorno mapeado el robot va creando un mapa local con el cuál es capaz de esquivar los objetos. Al llegar a la posición indicada el robot deberá elegir qué maleta es la correcta por medio del diálogo, escuchando y orientandose hacia la maleta correspondiente. Posteriormente debía seguir al árbitro mediante navegación a través de TF's publicadas en la posicion del árbitro estableciendo de esta forma un camino desde el robot hasta la persona.

### 1.1 ¿Cuál fue nuestra idea principal?

Para esta primera prueba pensamos que la mejor opción era crear un nuevo programa de navegación usando TF's de forma que siguiese a una persona enviándole la posición de esta. Además, podríamos usar este método de navegación para un entorno no mapeado, de forma que el robot siguiese las TF's indicadas y esquivara objetos usando el láser como en el bump&go. 
Para la solucionar el problema de la navegación al volver ideamos dos métodos:
- 1. Mapear la zona desconocida en un mapa local al mismo tiempo que seguíamos a la persona para así poder volver por una zona mapeada sin necesidad de preocuparnos por objetos locales.
- 2. Crear una especie de sendero de checkpoints de forma que el robot solo tuviese que ir de uno en uno para encontrar el camino de vuelta a la vez que usaba el láser para esquivar obstáculos.
  
Tras varios días creando el programa base de navegación y muchos imprevistos comenzamos a cuestionarnos estas dos soluciones, por lo que decidimos dejar parada la forma de volver.

Para la elección de equipaje la idea principal era hacerlo por diálogo.
  
### 1.2 ¿Cómo tuvimos que modificarlo?

Los últimos días seguíamos con problemas en la navegación por lo que creamos un programa que simplemente se movía al punto enviado.
Además, tuvimos que abandonar la idea de hacer esta prueba entera y creamos un nuevo Behavior Tree, el cuál navegaba hasta el punto del árbitro.

### 1.3 Relación entre plan y ejecución

Tras muchas horas de trabajo no pudimos completar la navegación por entornos desconocidos, seguidos de problemas con el Behavior Tree, además de inconvenientes en la detección y seguimiento de una persona. Es por ello que tuvimos que abandonar esta prueba y dejarla a medias.
En su ejecución el robot únicamente fue a la posición del arbitro.

## 2. Prueba 2
![Find My Mates BT](https://user-images.githubusercontent.com/90764439/167626112-21dcb8b3-6979-4942-9f2e-0e98babb363f.png)

### 2.0 Explicación de código

El robot espera la orden de start para comenzar, después, va yendo a cada posición mandada para buscar personas, si las encuentra deberá preguntarles el nombre, detectar por boundingboxes qué objeto portan y además detectar mediante un filtro de color, el color de su ropa. Tras guardar estos datos el robot debe ir a la posición del árbitro y decirle los datos recogidos.

### 2.1 ¿Cuál fue nuestra idea principal?

La idea principal era utilizar el programa de navegación base que ya intentamos implementar en la 1ª prueba, pero no fue posible. Teníamos pensado ir buscando punto por punto y crear un servicio para el habla y escucha del robot, el cuál finalmente no llegó a implementarse. Para detectar el color de la camiseta se implementó un filtro de color que mide los valores hsv del bounding box de la persona y realiza la media para devolver el color predominante en forma de string.
En cuanto al objeto, se detectaba mediante Darknet. 

### 2.2 ¿Cómo tuvimos que modificarlo?

Para esta prueba se nos ocurrió un Behavior Tree muy sencillo implementando la navegación de punto a punto, por lo que aún teniendo poco tiempo no tuvimos muchos problemas en la creación de los nodos. El diálogo fue implementado a través de nodos, de esta forma solo escuchaba cuando tenía que hacerlo. Y en caso de haber algún problema en la comprensión, podía volver a preguntar.

En resumen, el robot debía ir punto a punto mirando a ver si detectaba una persona, si lo hacía le tendría que preguntar su nombre, detectar el color de su camiseta con un filtro de color y el objeto que sujetaba y guardar todas estas respuestas para posteriormente ir a la posición del árbitro y decirle estos datos recogidos. Este proceso tenía que repetirse tres veces.

### 2.3 Relación entre plan y ejecución

Sin contar el error recurrente en la navegación, recibía bien los datos de las personas y los comunicaba de forma relativamente precisa, siendo el apartado más problematico la detección del objeto.
Finalmente, durante la ejecución un problema con la navegación hizo que el robot dejara de moverse, pero este siguió con el programa principal, haciendo preguntas y guardando las respuestas.

## 3. Prueba 3
![Recepcionist BT](https://user-images.githubusercontent.com/90764439/167626141-4266faec-8c6b-4725-a582-be29ea6c66dd.png)

### 3.0 Explicación de código

El robot debe esperar la orden de start para comenzar, tras recibirla, irá a por el primer invitado, preguntándole su nombre y bebida favorita y le guiará hasta la zona de sillas, allí, le preguntará la edad, después buscará un asiento vacío por medio de boundingboxes y le ordenará sentarse, teniendo en cuenta la edad, este proceso se repite hasta tres veces.

### 3.1 ¿Cuál fue nuestra idea principal?

Esta prueba no entraba dentro de nuestros planes, pero al no poder realizar la primera prueba correctamente y tener varias ideas de cómo realizarla decidimos intentarlo.
Lo primero era usar nuestro programa de navegación punto a punto para poder ir de la zona de sillas a la entrada varias veces.
Nuestro robot debía realizar una serie de preguntas al invitado y después llevarle a la zona de sillas, preguntándole a este su edad, de forma que si superaba un límite de edad de 50 años, se le recomendaría sentarse en el sofá.

### 3.2 ¿Cómo tuvimos que modificarlo?
 
 Para la elección de sillas vacías ideamos dos opciones:
 - 1. Crear un array con el nº de sillas correspondiente y tener booleanos que indicasen si habíamos sentado a alguien ahí y si eran sillas o sofás.
 - 2. Usar las boundingboxes, de forma que una silla vacía la detectase como silla, y una silla ocupada la detectaría como persona.
 Tras pensar alternativas para poder terminar este programa decidimos detectar las sillas vacías con boundingboxes y ordenar que se sentaran con diálogo.
 
### 3.3 Relación entre plan y ejecución

Al igual que en la seguda prueba, un fallo de navegación impidió que el robot se moviera. Sin embargo, continuó con la ejecución del programa, recibiendo y reportando los datos correctamente.

## 4. Problemas Generales

Nuestro robot tuvo problemas en la navegación, intentamos disminuir estas zonas negras del mapa y aumentar las grises para disminuir la velocidad del robot, pero en el segundo intento nos dio un fallo inesperado y no pudimos arreglarlo.
