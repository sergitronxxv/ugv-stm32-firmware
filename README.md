# UGV STM32 Firmware

Firmware para una placa **STM32F405RGT6** (generado con STM32CubeIDE) que traduce las señales PWM de salida de una controladora de vuelo Ardupilot/Ardurover (Kakute H743 Wing) a tramas CAN, siguiendo el protocolo **CANopen** y el perfil de dispositivo **CiA 402**, para el control de los dos nodos motor del vehículo.

Este firmware forma parte del Trabajo de Fin de Grado *"Desarrollo de un UGV autónomo con control remoto y navegación basada en ROS2"*.

## Función principal

1. Mide el ancho de pulso de las dos señales PWM de entrada (avance/retroceso y giro) mediante captura por flancos en el temporizador TIM2.
2. Convierte ambas señales a valores de velocidad en el rango -255 a 255, aplicando una zona muerta alrededor del valor neutro.
3. Aplica una mezcla diferencial (lógica de tanque) para calcular la velocidad de cada oruga a partir del avance y el giro solicitados.
4. Empaqueta el resultado en una trama CAN periódica que la ECU del vehículo interpreta directamente.
5. Gestiona el arranque de los nodos motor mediante una trama NMT de difusión y la secuencia de habilitación CiA 402 (Fault Reset → Shutdown → Switch On → Enable Operation).

## Periféricos empleados

| Periférico | Función |
|---|---|
| TIM2 (canales 2 y 3) | Captura del ancho de pulso de las dos señales PWM de entrada |
| CAN2 | Bus CAN efectivo: envío de velocidades y secuencia de habilitación CiA 402 |
| CAN1 | Reservado para una futura ampliación, sin uso activo en esta versión |
| GPIO | LED de estado |

## Estructura del repositorio

- `main.c` — bucle principal, inicialización de periféricos, captura de PWM, mezcla diferencial y gestión CiA 402.

## Documentación relacionada

El funcionamiento detallado de este firmware se describe en los apartados 5.2.1, 5.2.2 y en el Anexo C de la memoria del TFG.
