 
 # Introdução ROS

 ## 1. O que é ROS
A sigla ROS significa ***Robot Operating System***, em tradução livre é sistema operacional de robôs, sendo assim pode ser classificado como uma coleção de ferramentas, bibliotecas e convenções  que visam simplificar a criação de um robô complexo que utiliza uma grande variedade de plataformas.

Construir um software para um robô interagir corretamente com o as variações que acontecem no mundo real é uma tarefa muito complexa, e por esse motivo o ROS foi criado como uma "plataforma" colaborativa para esse desenvolvimento.

O ROS é contituido de vários programas, chamados de nós, rodando simultaneamente e se comunicando entre si.

 ### 1.1 *roscore*
 *roscore* é um programa servidor que permite essa comunicação entre os nós, cada um deles se conecta ao servidor com as informações que deseja publicar ou receber e ele providencia o endereço do nó que recebe ou manda essa informação, portanto todo sistema ROS precisa que esteja rodando o *roscore*. 

 Além disso o *roscore* providencia parametros do servidor que são utilizados pelos nós para configuração interna de estruturas de dados, parametros para códigos, parametros do robô, estre outros fatores. Para a configuração desses parâmentros pode-se usar uma linha simples de código *rosparam*

 ### 1.2 Pacotes
O sistema do ROS é organizado em pacotes, que são caracterizados por uma junção de programas dados e documentação. Existem centenas de pacotes distribuidos por repositorios abertos.

 ### 1.3 Nós
 Os nós são basicamente códigos individuais que rodam dentro dos pacotes, eles são responsáveis por definir qual é a mensagem publicada/lida e qual o destino ou origem.

 ### 1.4 Tópicos
 Os tópicos são o jeito mais comuns que os nós utilizam para sua intercomuniação, eles podem ser definidos como um ambiente para mensagens de um tipo determinado. Esses tópicos implementam uma comunicação de *publishe* e *subscribe*, o que significa que os nós podem publicar mensagens neles ou se subscrever a eles, assim retirando informações anteriormente colocadas. Esses tópicos são definidos dentro dos nós antes de fazerem algum tipo de comunicação

 ### 1.5 Services

São outra forma de comunicação com o robô. Eles permitem codificar uma funcionalidade específica para o robô e a oferece para qualquer um que chame esse serviço. liDiferentemente dos tópicos, os serviços são estruturados em duas partes – o ‘Service Server’, o qual disponibiliza a funcionadade para qualquer um que deseja utilizá-la, e o ‘Service Cliente’, o qual chama a funcionalidade do serviço.
- Exemplo: Executar `$ roslaunch service_demo servisse_launch.launch` em um WebShell (terminal online do ROS) e, depois, `$ rosservice call /servisse_demo “{}”` em outra WebShell, pois o ‘Service Server’ deve continuar rodando para poder ser chamado posteriormente.

### 1.6 Actions

São similares a services no sentido de que permitem que seja codificada uma funcionalidade para ser utilizada posteriormente. No entanto, diferentemente dos services, nos quais o robô deve terminar de executar o servisse para fazer qualquer outra coisa, em actions é possível que o robô esteja efetuando outro comando enquanto executa uma action.