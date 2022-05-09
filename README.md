[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-f059dc9a6f8d3a56e377f745f24479a46679e63a5d9fe6f495e02850cd0d8118.svg)](https://classroom.github.com/online_ide?assignment_repo_id=7421157&assignment_repo_type=AssignmentRepo)
# RoboCup

## ¿Qué pruebas elegimos y por qué?
Al comenzar a planear el funcionamiento del kobuki en cada una de nuestras pruebas pensamos y decidimos que las más sencillas serían la 1ª y la 2ª, ya que creando un buen programa base de navegación tendríamos todo casi hecho.

## Prueba 1

### ¿Cuál fue nuestra idea principal?

Para esta primera prueba pensamos que la mejor opción era crear un nuevo programa de navegación usando tfs de forma que siguiese a una persona enviándole la posición de esta. Además, podríamos usar este método de navegación para un entorno no mapeado, de forma que el robot siguiese las tfs indicadas y esquivar objetos usando el láser como en el bum&go. 
Para la solucionar el problema de la navegación al volver ideamos dos métodos:
  1- Mapear la zona desconocida en un mapa local al mismo tiempo que seguíamos a la persona para así poder volver por una zona mapeada sin necesidad de preocuparnos    por objetos locales.
  2- Crear una especie de sendero de checkpoints de forma que el robot solo tuviese que ir de uno en uno para encontrar el camino de vuelta a la vez que usaba el láser para esquivar obstáculos.
  
Tras varios días creando el programa base de navegación y muchos imprevistos comenzamos a cuestionarnos estas dos soluciones, por lo que decidimos mapear todo el recorrido por el momento.

Para la elección de equipaje la idea principal era dejarlo al azar, pero tras unas horas probando ideamos un filtro de color con el que podíamos distinguir los colores de personas, objetos, etc.
  
### ¿Cómo tuvimos que modificarlo?

Los últimos días seguíamos con problemas en la navegación por lo que creamos un programa que simplmente se movía al punto enviado creando una tf hacia este.
Además, tuvimos que abandonar la idea de hacer esta prueba entera y creamos un nuevo BT, el cuál navegaba hasta el punto del árbitro y elegía una maleta con el filtro de color.

### Relación entre plan y ejecución

La verdad es que esta prueba ha sido a la que más horas y esfuerzo hemos dedicado y la que peor logramos, nos surgieron muchos problemas con la naegación, seguidos de problemas con el BT, además de inconvenientes en la detección y seguimiento de una persona. Es por ello que tuvimos que abandonar esta prueba y dejarla a medias.


## Prueba 2

### ¿Cuál fue nuestra idea principal?

### ¿Cómo tuvimos que modificarlo?

### Relación entre plan y ejecución

## Prueba 3

### ¿Cuál fue nuestra idea principal?

### ¿Cómo tuvimos que modificarlo?

### Relación entre plan y ejecución
