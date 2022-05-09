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
      - 1.1 ¿Cuál fue nuestra idea principal?
      - 1.2 ¿Cómo tuvimos que modificarlo?
      - 1.3 Relación entre plan y ejecución
   - 2. Prueba 2
      - 2.1 ¿Cuál fue nuestra idea principal?
      - 2.2 ¿Cómo tuvimos que modificarlo?
      - 2.3 Relación entre plan y ejecución
   - 3. Prueba 3
      - 3.1 ¿Cuál fue nuestra idea principal?
      - 3.2 ¿Cómo tuvimos que modificarlo?
      - 3.3 Relación entre plan y ejecución
   - 4. Problemas Generales
   

## 0.0 ¿Qué pruebas elegimos y por qué?
Al comenzar a planear el funcionamiento del kobuki en cada una de nuestras pruebas pensamos y decidimos que las más sencillas serían la 1ª y la 2ª, ya que creando un buen programa base de navegación tendríamos todo casi hecho.

## 1. Prueba 1

### 1.1 ¿Cuál fue nuestra idea principal?

Para esta primera prueba pensamos que la mejor opción era crear un nuevo programa de navegación usando tfs de forma que siguiese a una persona enviándole la posición de esta. Además, podríamos usar este método de navegación para un entorno no mapeado, de forma que el robot siguiese las tfs indicadas y esquivar objetos usando el láser como en el bum&go. 
Para la solucionar el problema de la navegación al volver ideamos dos métodos:
- 1. Mapear la zona desconocida en un mapa local al mismo tiempo que seguíamos a la persona para así poder volver por una zona mapeada sin necesidad de preocuparnos    por objetos locales.
- 2. Crear una especie de sendero de checkpoints de forma que el robot solo tuviese que ir de uno en uno para encontrar el camino de vuelta a la vez que usaba el láser para esquivar obstáculos.
  
Tras varios días creando el programa base de navegación y muchos imprevistos comenzamos a cuestionarnos estas dos soluciones, por lo que decidimos mapear todo el recorrido por el momento.

Para la elección de equipaje la idea principal era dejarlo al azar, pero tras unas horas probando ideamos un filtro de color con el que podíamos distinguir los colores de personas, objetos, etc.
  
### 1.2 ¿Cómo tuvimos que modificarlo?

Los últimos días seguíamos con problemas en la navegación por lo que creamos un programa que simplmente se movía al punto enviado creando una tf hacia este.
Además, tuvimos que abandonar la idea de hacer esta prueba entera y creamos un nuevo BT, el cuál navegaba hasta el punto del árbitro y elegía una maleta con el filtro de color.

### 1.3 Relación entre plan y ejecución

La verdad es que esta prueba ha sido a la que más horas y esfuerzo hemos dedicado y la que peor logramos, nos surgieron muchos problemas con la naegación, seguidos de problemas con el BT, además de inconvenientes en la detección y seguimiento de una persona. Es por ello que tuvimos que abandonar esta prueba y dejarla a medias.
En su ejecución el robot fue a la posición del arbitro y ahí terminó su ejecución.

## 2. Prueba 2

### 2.1 ¿Cuál fue nuestra idea principal?

La idea principal era utilizar el programa de navegación base que ya intentamos implementar en la 1ª prueba, pero no fue posible. Teníamos pensado ir buscando punto por punto y crear un servicio para el habla y escucha del robot, lo cuál también se nos complicó.

### 2.2 ¿Cómo tuvimos que modificarlo?

Para esta prueba se nos ocurrió un BT muy sencillo implementando la navegación de punto a punto, por lo que aún teniendo poco tiempo no tuvimos muchos problemas en la creación de los nodos. Al tener que rehacer el habla se nos ocurrió hacerlo por nodos, de esta forma solo escuchaba cuando tenía que hacerlo.
En resumen, el robot debía ir punto a punto mirando a ver si detectaba una persona, si lo hacía le tendría que hacer una serie de preguntas y guardar todas estas respuestas para posteriormente ir a la posición del árbitro y decirle estos datos recogidos. Este proceso solo tenía que repetirse tres veces.

### 2.3 Relación entre plan y ejecución

Esta prueba nos funcionaba perfectamente, pero la navegación nunca es perfecta y podía alterarnos la ejecución ya que en el simulador funcionaba perfectamente, pero en la realidad podía desorientarse.
Finalmente, el robot entró en una zona negra de su mapa, por lo que la navegación murió y siguió con el programa principal, haciendo preguntas y guardando las respuestas.

## 3. Prueba 3

### 3.1 ¿Cuál fue nuestra idea principal?

Esta prueba no entraba dentro de nuestros planes, pero al no poder realizar la primera prueba correctamente y tener varias ideas de cómo realizarla decidimos intentarlo.
Lo primero era usar nuestro programa de navegación punto a punto para poder ir de la zona de sillas a la entrada varias veces.
Nuestro robot debía realizar una serie de preguntas al invitado y después llevarle a la zona de sillas, preguntándole a este su edad, de forma que si superaba un límite de edad se le recomendaría sentarse en el sofá.

### 3.2 ¿Cómo tuvimos que modificarlo?
 
 Como esta prueba la cogimos a última hora, no tuvimos que modificarlo, simplemente planeamos dos opciones para la elección de sillas vacías:
 - 1. Crear un array con el nº de sillas correspondiente y tener booleanos que indicasen si habíamos sentado a alguien ahí y si eran sillas o sofás.
 - 2. Usar las boundingboxes, de forma que una silla vacía la detectase como silla, y una silla ocupada la detectaría como persona.
 Tras pensar alternativas para poder terminar este programa decidimos simplemente ordenar que se sentaran en una silla o sofá, dependiendo de la edad.
 
### 3.3 Relación entre plan y ejecución

En la ejecución de este último programa nos pasó lo mismo que en la segunda prueba, el robot entró en una zona negra del mapa, por lo que su navegación murió.

## 4. Problemas Generales

Nuestro robot tuvo problemas en la navegación ya que al entrar en una zona negra ya no era capaz de salir, intentamos disminuir estas zonas negras del mapa y aumentar las grises para disminuir la velocidad del robot, pero en el segundo intento nos dio un fallo inesperado y no pudimos arreglarlo.
Para arrancar nuestro programa el robot debía escuchar la orden start, go, etc, pero no reconocía bien start, y perdimos un valioso tiempo.
