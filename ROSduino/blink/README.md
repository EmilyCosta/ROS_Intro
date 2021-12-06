# Blink Explicado

O código que será explicado por partes é o seguinte:

 ```cpp
 #include <ros.h>
 #include <std_msgs/Empty.h>
 #include <std_msgs/String.h>

 ros::NodeHandle nh;

 #define LED_PIN 13

 void messageCb( const std_msgs::Empty &toggle_msg );

 std_msgs::String str_msg;
 ros::Publisher pdp("pdp", &str_msg);
 ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb);

 uint8_t led_state = 0;

 void setup() {
     pinMode(LED_PIN, OUTPUT);
     nh.initNode();
     nh.subscribe(sub);
     nh.advertise(pdp);
 }

 void loop() {
     nh.spinOnce();
     delay(1);
 }

 void messageCb( const std_msgs::Empty &toggle_msg) {
     led_state ^= 1;
     digitalWrite(LED_PIN, led_state);
     sprintf(str_msg.data, "%s", led_state ? "H" : "L");
     pdp.publish( &str_msg );
 }
 ```

 ## Comandos Iniciais

Inicialmente, é necessário importar alguns módulos

 ```cpp
 #include <ros.h>
 #include <std_msgs/Empty.h>
 #include <std_msgs/String.h>
 ```

Agora, serão criadas as variáveis: uma handle - 'nh' - para gerenciar os nós, a variável 'led_state' para, depois, ser utilizada na função callback e a 'str_msg', a qual será a mensagem publicada.

 ```cpp
 ros::NodeHandle nh;
 uint8_t led_state = 0;
 std_msgs::String str_msg;
 ```

O próximo passo é definir qual porta digital será utilizada. No caso, será utilizada a 13, a qual corresponde ao LED do próprio arduino

 ```cpp
 #define LED_PIN 13
 ```

Para finalizar a etapa inicial, serão definidos os nós (Publisher e Subscriber) que serão utilizados

 ```cpp
 ros::Publisher pdp("pdp", &str_msg);
 ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb);
 ```

 - O publisher utilizado será o 'pdp' e será publicada a variável `str_msg`
 - O subscriber 'sub' lerá o tópico 'toggle_led' e executará a função callback

 ## Função Callback

Basicamente, a função callback será encarregada de ler os valores publicados no tópico a cada mudança de valor que ocorrer neste.
 Antes de definir a função, deve-se anunciá-la:

 ```cpp
 void messageCb( const std_msgs::Empty &toggle_msg );
 ```

Agora sim a função pode ser definida:

 ```cpp
 void messageCb( const std_msgs::Empty &toggle_msg) { 
     led_state ^= 1;
     digitalWrite(LED_PIN, led_state);
     sprintf(str_msg.data, "%s", led_state ? "H" : "L");
     pdp.publish( &str_msg );
 }
 ```

- O nome da função é 'messageCb'
- `std_msgs::Empty` é um tipo de variável, assim como `int` e `str`
- `toggle_msg`é o tópico que o subscriber lerá
- `^=` inverte o valor (de 1 para 0 e de 0 para 1)
- `digitalWrite(onde, o que)` executa um comando no arduino
- `sprintf`irá printar 'H' (HIGH - 1 - LED aceso) ou 'L' (LOW - 0 - LED apagado) com base na variável `led_state`
- `pdp.publish`: o nó 'pdp' irá publicar 'H' ou 'L' no tópico `toggle_msg`

## Programa Principal

Começaremos pelo setup:

 ```cpp
 void setup() {
     pinMode(LED_PIN, OUTPUT);
     nh.initNode();
     nh.subscribe(sub);
     nh.advertise(pdp);
 }
 ```

- `pinMode` irá definir se o pino inserido é entrada ou saída. No caso, será uma saída ('OUTPUT')
- `nh.initNode` inicia os nós
- `nh.subscribe(pdp)` indica qual subscriber será utilizado
- `nh.advertise(pdp)` deve-se anunciar a utilização posterior de um publisher

O loop principal em si é simples:

 ```cpp
 void loop() {
     nh.spinOnce();
     delay(1);
 }
 ```

- `nh.spinOnce()` faz passar uma vez pelos nós
- O comando `delay()`faz com que o programa dê uma pausa determinada (em milisegundos)

 
