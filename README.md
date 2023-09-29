
ESTRATEGIA
He requerido de mucha investigación para poder llevar a cabo este proyecto, buscar una radio con un buen alcance, velocidad de datos, interferencias, etc. Arduino para tener la libertad de programar como mejor me venga. Mando propio, lo que me permite añadir tantos modos como quiera, mayor control sobre el funcionamiento y el precio, ya que mando y receptor suelen ser bastante caros, haciendo el mío he conseguido que salga muchísimo mas barato.


![DSC09757-1024x1536](https://github.com/fadriqueat/Avion/assets/74105814/bdcd1d00-9d14-41e9-a3cf-8f56017dd596)



DECISIONES
Mi idea siempre ha sido hacer un planeador, para lo cual el motor es ideal (con ESC de 30A) y hélices de planeador. La batería podría haber sido una mejor (batería de 3s con 3000/4000mAh). Los servos no son lo mejor que hay, pero me sirve para asegurarme de que todo funciona, cabe la posibilidad de comprar unos mejores (con engranajes metálicos y no de plástico) en un futuro. He decidido comprar la impresora 3D, una Creality Ender 3 V2 Neo, ya que me permite hacer todo tipo de aviones en un futuro por un precio relativamente bajo, comprar un avión nuevo sale extremadamente caro, y chocarlo y romperlo es algo muy común, por lo que hacer uno nuevo entero por 10 euros sale muy bien. En cuanto al peso, la diferencia entre LW-PLA y FOAM no se nota casi, con PLA si, ya que esta pesa casi el doble. Ya tenía pensado comprar una impresora 3D mas adelante, para posibles proyectos en el futuro.


![DSC09758-1024x683](https://github.com/fadriqueat/Avion/assets/74105814/aea257c7-d150-468e-8a68-98fadfb5ee2e)


![DSC09756-1024x683](https://github.com/fadriqueat/Avion/assets/74105814/a2c672d8-f30c-4863-85a2-7cbd5aa84fba)

  MEJORAS
+ La bateria de 2s no suministraba la suficiente potencia al motor, por lo que he pasado a una Lipo 3s con 2200mAh

+ Viendo que mis habilidades para volar el avión dejan que desear, he añadido un IMU (Inertial Measurement Unit). El MPU6050, con el objetivo de estabilizar el avión en vuelo. Tambien he aprovechado para incluir un botón SAFE y otro para cambiar los modos de vuelo




The required components for this code are:

  - Arduino nano
  - Nrf24L01 radio module
  - MPU6050
  - OLED screen
  - Joysticks & Buttons
  - Servos & Motor (with ESC)
  
If someone sees im using their code ill gladly tag them.


DETALLES
Modelo eclipson model A (sin tren de aterrizaje) – https://www.eclipson-airplanes.com/es/modela
Arduino Nano x2 – https://www.amazon.es/gp/product/B078SBBST6/ref=ppx_yo_dt_b_asin_title_o04_s00?ie=UTF8&psc=1
Radio Nrf24L01 x2 – https://www.amazon.es/gp/product/B07PBBC4H9/ref=ppx_yo_dt_b_asin_title_o04_s01?ie=UTF8&psc=1
Motor brushless – https://www.amazon.es/gp/product/B07BS7DFW3/ref=ppx_yo_dt_b_asin_title_o04_s00?ie=UTF8&psc=1
Haisito cargador – https://www.amazon.es/gp/product/B07SS4VWSS/ref=ppx_yo_dt_b_asin_title_o03_s00?ie=UTF8&psc=1
Giroscopio MPU-6050 – https://www.amazon.es/gp/product/B07TKLYBD6/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1
Servo MG90S x3 – https://www.amazon.es/gp/product/B086V7TXXC/ref=ppx_yo_dt_b_asin_title_o04_s00?ie=UTF8&psc=1
Bateria LiPo 3s – https://www.amazon.es/gp/product/B07LFVDB1Y/ref=ppx_yo_dt_b_asin_title_o05_s00?ie=UTF8&psc=1
Pantalla OLED – https://www.amazon.es/gp/product/B01L9GC470/ref=ppx_yo_dt_b_asin_title_o06_s00?ie=UTF8&psc=1
Impresora Ender 3 V2 Neo – https://www.amazon.es/gp/product/B0B5CCPD67/ref=ppx_yo_dt_b_asin_title_o02_s00?ie=UTF8&psc=1
Filamento PLA+ – https://www.amazon.es/gp/product/B07R8X76GW/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1
Cable carga arduino x2 – https://www.amazon.es/gp/product/B08BCNPNJ7/ref=ppx_yo_dt_b_asin_title_o05_s00?ie=UTF8&psc=1
Gomas elasticas – https://www.amazon.es/gp/product/B01J1N2EBO/ref=ppx_yo_dt_b_asin_title_o06_s00?ie=UTF8&psc=1
Cinta adhesiva
Varillas
Pegamento
