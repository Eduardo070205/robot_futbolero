# MGApp en Windows 11 con iPhone 16 Pro

## Preparación de Python

Usa Python 3.13 de 64 bits (ya se detectó instalado en este equipo). Abre
PowerShell y ejecuta estos comandos, uno por uno:

```powershell
cd "C:\Users\maria\OneDrive\Desktop\Codigos_Proyecto-VSSS\proyecto-VSSS"
py -3.13 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe MGApp.py
```

Después de la primera instalación, para volver a abrirlo basta con entrar a
la carpeta y ejecutar el último comando. No necesitas activar el entorno ni
cambiar la política de ejecución de PowerShell. Tkinter viene con el instalador
oficial de Python para Windows. Instala `opencv-python`, sin la variante
`headless`, porque la calibración necesita ventanas.

## Opción recomendada: iPhone por USB con DroidCam

1. Instala **DroidCam Webcam & OBS Camera** en el iPhone y el **DroidCam Client**
   para Windows desde https://droidcam.app/.
2. Conecta el iPhone con un cable USB-C de datos, desbloquéalo y acepta
   **Confiar en esta computadora** cuando aparezca.
3. Abre DroidCam en ambos equipos. En el cliente de Windows, agrega una fuente
   DroidCam y selecciona el iPhone; actualiza la lista de dispositivos si hace
   falta. Primero confirma que puedes ver el video en el cliente.
4. Mantén abierto DroidCam Client y activa su salida de cámara virtual si tu
   versión lo requiere. Si Windows no detecta el iPhone, revisa los controladores
   Apple USB / Apple Mobile Device Support según la guía oficial:
   https://droidcam.app/help/#usb.
5. En Windows: **Configuración → Privacidad y seguridad → Cámara**, habilita
   el acceso a la cámara y el acceso para aplicaciones de escritorio.
6. En MGApp elige **Local / iPhone USB** en Fuente de cámara. Prueba el índice
   de cámara 0, 1, 2, etc. hasta ver la imagen del iPhone. El número depende de
   las cámaras instaladas y puede cambiar al conectar dispositivos.

El iPhone necesita la aplicación y el cliente para aparecer como webcam en
Windows. Conectar únicamente el cable no basta. Esta ruta también permite usar
otros clientes que expongan una cámara virtual de Windows, aunque la validación
con tu controlador concreto requiere conectar el teléfono.

Para empezar, usa cámara trasera 1x, orientación horizontal, imagen sin espejo y
30 FPS. Prueba 1280×720 si está disponible en tu versión/licencia. DroidCam ofrece
funciones básicas gratuitas; algunas opciones HD o controles requieren Pro.
Mantén la lente, el zoom, la orientación y los ajustes fijos después de calibrar.
El enfoque y el balance de blancos del iPhone se ajustan en la app del teléfono
o su cliente, según las funciones disponibles.

## Alternativa: video por Wi-Fi / URL

En una app que exponga video HTTP(S) o RTSP, usa la dirección de transmisión
exacta que indique la app. Ambos equipos deben poder comunicarse por la red.
En DroidCam, la documentación describe la forma:

```text
http://IP_DEL_IPHONE:4747/video
```

En MGApp elige **iPhone URL**, escribe la dirección y pulsa **Conectar cámara
iPhone**. La dirección de ejemplo no es una IP real: sustitúyela por la indicada
en tu teléfono. Usa la URL de video; una página de controles o una vista web no
equivale a un flujo de video. MGApp aplica límites de espera de 5 s al abrir y
2 s por lectura de red. La conexión no se reabre automáticamente si se corta:
restablece el video en el teléfono y vuelve a pulsar el botón.

Por USB se evita la interferencia Wi-Fi; es la opción preferida para empezar.
Referencia oficial de conexión y URL: https://droidcam.app/help/.

## Calibración y primer uso

1. Comienza sin conectar el ESP32 maestro: MGApp arranca en modo simulación
   (procesa la cámara sin enviar a motores; no es un simulador físico).
2. Coloca el iPhone fijo encima del campo. Pulsa **Calibración** y luego
   **Elegir las 4 esquinas otra vez**: superior izquierda, superior derecha,
   inferior derecha e inferior izquierda. Se guardan en `cancha_config.json`.
   Los puntos que venían en el proyecto corresponden a otra toma.
3. En esa ventana pulsa **Recalibrar los 6 colores**, o usa **Auto colores**
   desde la ventana principal. Selecciona cada color solicitado con un clic,
   ajusta sus barras y confirma con **g** o **Espacio**. **Esc** cancela ese color.
   Auto colores abre la calibración interactiva; requiere tus clics.
   Al completar los seis colores se guarda `lab_calibration.json`.
4. Cierra la calibración. MGApp reabre la cámara y recarga los archivos.
   Confirma que detecta pelota, POR y DEL; selecciona uniforme y portería
   propia. Si cambias brillo, contraste, exposición, cámara o iluminación,
   vuelve a comprobar los colores. Si mueves o giras el teléfono, recalibra
   también las esquinas.
5. Para motores, conecta el ESP32 maestro por un cable USB de datos. Pulsa
   **Detectar**, elige su puerto **COM** y cierra el monitor serie de Arduino
   antes de pulsar **Conectar**. Se utiliza 921600 baudios, igual que en
   `Maestro.ino`. **Conectar permite enviar movimiento**: pulsa Pausa antes
   si aún estás preparando los robots y luego Reanudar cuando estén listos.
6. Maestro y esclavos deben tener sus programas cargados, las MAC correctas
   y el mismo canal ESP-NOW (2). No se modificó su firmware.

MGApp reenvía comandos constantes cada 100 ms aproximadamente cuando se procesan
fotogramas. Si el video o el procesamiento se interrumpen, el esclavo conserva
su parada tras 250 ms sin paquetes. Cierra MGApp con la X para enviar una orden
de parada y liberar el puerto.

`camera_calib.json` es opcional y no viene incluido: sin él, deja desactivada
la corrección de distorsión radial.

## Equipo de dos robots

Los roles activos son **POR** y **DEL**. El delantero usa los marcadores
**verde + rosa** y asume cobertura defensiva, presión y ataque. El portero
mantiene sus marcadores: **azul + rosa** si el uniforme es amarillo, o
**amarillo + rojo** si el uniforme es azul. Los marcadores rojo + verde del
antiguo mediocampista ya no asignan un rol propio.

Si la pelota está en el tercio propio y a más de 220 píxeles del delantero,
DEL vuelve a cubrir el borde de ese tercio siguiendo su posición vertical.
Cuando se acerca, presiona para recuperarla; en campo rival ataca. El objetivo
del delantero se mantiene fuera del área propia, reservada al portero.
El HUD indica COBERTURA, PRESION o ATAQUE.

Se conserva el formato serial `<POR|0,0|DEL>` para usar el firmware actual:
el antiguo canal MED recibe siempre una orden de parada. No hay que volver
a cargar los programas de los ESP32 por este cambio. Se mantienen los seis
colores de calibración porque también sirven para el portero y los rivales.

## Si no aparece video

- Confirma primero la imagen en DroidCam y prueba otro índice en MGApp.
- Cierra otras aplicaciones que estén consumiendo la cámara virtual.
- Revisa permisos de cámara de Windows y del iPhone.
- Para URL: revisa la IP actual, que la app esté transmitiendo y que la red
  permita la conexión. No necesitas desactivar el firewall de Windows.
- Si falla un COM, revisa el Administrador de dispositivos, el cable y el driver
  del adaptador USB del ESP32 (por ejemplo CH340 o CP210x, según tu placa).

## Comprobaciones de software

```powershell
.\.venv\Scripts\python.exe -B -m unittest discover -s tests -v
```

Las pruebas simulan cámaras y puertos; no sustituyen la validación con el
iPhone, el ESP32 y los motores conectados.
