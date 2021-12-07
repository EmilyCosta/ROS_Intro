 
 # Introdução ROS

 ## 1. O que é ROS
A sigla ROS significa ***Robot Operating System***, a qual, em tradução livre, significa "sistema operacional de robôs", sendo, então, classificada como uma coleção de ferramentas, bibliotecas e convenções. Tal sistema visa simplificar a criação de robôs complexos que utilizam uma grande variedade de plataformas.

Construir um software para um robô interagir corretamente com as variações que acontecem no mundo real é uma tarefa muito complexa. Diante disso, o ROS foi criado como uma "plataforma" colaborativa para esse desenvolvimento.

O ROS é contituido de vários programas - chamados de "nós" - rodando simultaneamente e comunicando-se entre si.

 ### 1.1 *roscore*
 *roscore* é um programa servidor o qual permite a comunicação entre os nós. Cada um dos nós, em posse das informações que deseja publicar ou receber, conecta-se ao servidor para que ele providencie o endereço do tópico que possui os dados desejados. Portanto, todo sistema ROS depende do *roscore* para trocar informação. 

 Além disso, o *roscore* fornece parâmetros do servidor, os quais são utilizados pelos códigos, pelo robô, pelos nós para configuração interna de estruturas de dados, dentre outros. Para a configuração desses parâmentros, pode-se recorrer a uma linha simples de código: *rosparam*.

 ### 1.2 Pacotes
O sistema do ROS é organizado em pacotes, que são caracterizados por uma junção de programas e documentação. Existem centenas de pacotes distribuidos por repositórios abertos.

 ### 1.3 Nós
 Os nós são, basicamente, códigos individuais que rodam dentro dos pacotes. Eles são responsáveis por definir qual é a mensagem publicada/lida e qual o destino ou origem.

 ### 1.4 Tópicos
 Os tópicos são o jeito mais comum que os nós utilizam para sua intercomuniação. Eles podem ser definidos como um ambiente para mensagens de um tipo determinado, dependendo do programa que envia tais informações. Esses tópicos implementam uma comunicação de *publisher* e *subscriber*, o que significa que os nós podem publicar mensagens neles ou se subscreverem a eles, ou seja, enviar dados para eles ou receber os dados ali presentes. Os tópicos para os quais serão enviadas mensagens ou dos quais serão retiradas informações são definidos dentro do programa de cada nó.