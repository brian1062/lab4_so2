# laboratorio 4 FreeRTOS

---

#### Asignatura "Sistemas Operativos 2"

Este proyecto utiliza [_FreeRTOS_](https://www.FreeRTOS.org) para implementar y gestionar cuatro tareas distintas en un microcontrolador el cual se emulado utilizando [qemu](https://www.qemu.org/). Se implementan varias tareas para simular la lectura de un sensor de temperatura, el filtrado de datos, la visualización en una pantalla LCD y la generación de estadísticas del sistema.

### Características

- **vSensorTask**: Lee datos de temperatura simulados y los envía a una cola para ser filtrados.
- **vFilterTask**: Filtra los datos de temperatura utilizando un promedio móvil y envía los datos filtrados a la tarea de visualización.
- **vGraphTask**: Visualiza los datos de temperatura en una pantalla LCD.
- **vStatsTask**: Genera y envía estadísticas sobre el uso de CPU y la pila de cada tarea a través de UART.
- **Interrupción UART**: Permite ajustar el tamaño de la ventana de filtrado a través de entradas UART.

---

## Requisitos

Asegúrate de tener los siguientes requisitos antes de empezar:

- [Make](https://www.gnu.org/software/make/)
- [qemu](https://www.qemu.org/)
- [Python](https://python-guide-es.readthedocs.io/es/latest/starting/install3/linux.html)

## Clonación del Repositorio

Para clonar el repositorio, abre tu terminal y ejecuta el siguiente comando:

```bash
git clone https://github.com/brian1062/lab4_so2.git
cd lab4_so2/FreeRTOS/Demo/CORTEX_LM3S811_GCC
```
Para compilar el proyecto:
```bash
make
```
Para Ejecutar el proyecto en un terminal:
```bash
qemu-system-arm -machine lm3s811evb -kernel gcc/RTOSDemo.axf -serial  pty
```
En caso de querer ejecutarlo en modo Debug:
```bash
qemu-system-arm -machine lm3s811evb -kernel gcc/RTOSDemo.axf -s -S
```
Para ver `vStatsTask` se creo un script en python el cual se puede ejecutar en otra terminal de la siguiente manera:
```bash
python3 ./com_ser.py <pty_num>
```
En esa misma terminal si ingresamos valores entre 0 10 podemos cambiar el numero de **size_N**
> [!WARNING]
> size_N debe ser un número entero mayor que 0 y menor a 10. Por defecto, size_N es 2.

---

### Tareas Implementadas

**1. vSensorTask**

Esta tarea simula la lectura de un sensor de temperatura y publica estos valores en una cola `xFilterQueue`. La temperatura se ajusta aleatoriamente dentro de un rango definido [18;34], asegurando que no supere los valores máximos o mínimos permitidos.

- Frecuencia: 10 Hz (cada 100 ms) pedida en la consigna.
- Funcionamiento:
  - Inicializa la última ejecución.
  - Lee y ajusta la temperatura aleatoriamente.
  - Publica la temperatura en `xFilterQueue`.

```c
static void vSensorTask( void *pvParameters )
{
    //Task Code
}
```

**2. vFilterTask**

Esta tarea recibe los valores de temperatura de `xFilterQueue`,
aplica un filtro de media móvil para suavizar los datos y
publica el valor filtrado en `xPrintQueue` para ser graficados.

- Funcionamiento:
  - Recibe la temperatura de `xFilterQueue`.
  - Actualiza la lista de valores con la nueva temperatura.
  - Aplica un filtro de media móvil para suavizar los datos.
  - Publica el valor filtrado en `xPrintQueue`.

```c

static void vFilterTask( void *pvParameters )
{
    //Task Code
}
```

**3. vGraphTask**

Esta tarea recibe el valor filtrado de la temperatura de `xPrintQueue` y actualiza un display gráfico para mostrar los valores de temperatura y el tamaño de la ventana del filtro asi como tambien graficarlos en el tiempo.

- Funcionamiento:
  - Recibe la temperatura filtrada de `xPrintQueue`.
  - Convierte la temperatura y el tamaño del filtro a ASCII para visualizarlos.
  - Dibuja los valores en el display gráfico.
  - Dibuja los ejes X e Y. \* Mapea la temperatura recibida en el grafico y la dibuja.
  ![Grafico generado por vGraphTask](/images/graph.png)

```c
static void vGraphTask( void *pvParameters )
{
    //Task Code
}
```

**4. vStatsTask**

Esta tarea recopila y envía estadísticas sobre el uso del stack y el tiempo de ejecución de las tareas del sistema a través del puerto UART.

- Frecuencia: Cada 2500 ms.
- Funcionamiento:
  - Toma un snapshot del estado del sistema.
  - Calcula el tiempo de ejecución y el uso del stack de cada tarea.
  - Envía esta información al puerto UART.

```c
static void vStatsTask(void *pvParameters)
{
    //Task Code
}
```

![Estadisticas generadas por vStatsTask](/images/image1.png)

#### Manejo de Interrupciones

Se utiliza una interrupción por UART para ajustar el valor de **size_N**, el cual determina el tamaño de la ventana del filtro en **vFilterTask**. Cuando se recibe un carácter por UART, este se convierte a entero y se actualiza **size_N**.

```c
void vUART_ISR(void)
{
    int rx;
    unsigned long ulStatus;
    ulStatus = UARTIntStatus( UART0_BASE, pdTRUE );
    UARTIntClear( UART0_BASE, ulStatus );

    if( ulStatus & UART_INT_RX )
    {
        rx = UARTCharGet(UART0_BASE) - '0';
        xQueueSend(xUartRQueue, &rx, portMAX_DELAY);
        if(xQueueReceive(xUartRQueue, &size_N, 0) != pdPASS){
            while (true){};
        }
    }
}
```


### Analisis de Stack

En primera instancia se uso una stack grande por default para todas las tareas, posteriormente gracias a la vStatsTask (de igual forma se puede hacer con gdb) y a la funcion uxTaskGetStackHighWaterMark se le seteo las siguientes stack para cada tarea:

```c
#define SENSOR_STACK_SIZE   	    ( ( unsigned short ) 33 )
#define GRAPH_STACK_SIZE	    ( ( unsigned short ) 60 )
#define FILTER_STACK_SIZE   	    ( ( unsigned short ) 48 )
#define STATS_STACK_SIZE	    ( ( unsigned short ) 50 )
```

Por lo cual nuestra tabla de tareas nos queda de la siguiente manera:
![Tabla de tareas con watermark](/images/image2.png)
Se aprecia como la StackFree de cada tarea es menor a la de la segunda imagen

## References

- [Generador Random](https://github.com/istarc/freertos/blob/master/FreeRTOS/Demo/CORTEX_A5_SAMA5D3x_Xplained_IAR/AtmelFiles/libboard_sama5d3x-ek/source/rand.c)
- [Estadisticas para vStatsTask](https://www.freertos.org/uxTaskGetSystemState.html)
- [Analisis de Stack](https://www.freertos.org/uxTaskGetStackHighWaterMark.html)
