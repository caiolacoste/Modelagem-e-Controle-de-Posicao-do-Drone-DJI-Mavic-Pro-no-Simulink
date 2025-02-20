# Modelagem e Controle de Posicao do Drone DJI Mavic Pro no Simulink

## Introdução

![image](https://github.com/user-attachments/assets/dfdc7c5a-3dd0-4a61-b5c5-09b96ddc6fac)

<!-- Tentando aprofundar os estudos sobre drones, vi a playlist oficial do Matlab sobre controle de drones, mas quando eles passam para o Simulink, é utilizado o `Aerospace Blockset`, que já vem com os blocos prontos e funciona para o minidrone da marca *Parrot*. -->

Este projeto tem como objetivo explicar o diagrama de blocos criado no Simulink para a simulação e controle do drone conhecido *DJI Mavic Pro*. O desenvolvimento foi dividido em duas partes. 

Primeiro foi feita a modelagem do comportamento do drone, reproduzindo a força e torque das hélices, assim como seu peso e forças externas como vento, resistência do ar.

Depois de ter o modelo da planta completo, capaz de retornar aceleração, velocidade e posição angulares e lineares, foi feito o controle de posição do drone, começando com a malha interna de controle de ângulo, garantindo que ele não vire de ponta-cabeça, e uma malha externa de controle de posição.

<!-- OBS: Este projeto é uma continuação do código fornecido no curso *Model, Simulate and Control a Drone in MATLAB & SIMULINK*, feito na Udemy, visto que depois de aplicar na prática, foram encontrados diversos erros na mixagem dos motores original fornecida, e o controle de posição não estava funcionando de forma alguma. -->

## Metodologia

![image](https://github.com/user-attachments/assets/e4b57b36-bc28-4966-a88b-6e62802e489e)



As informações obtidas do drone na internet são:
- Comprimento diagonal (sem hélices): 335mm;
- Massa: 0.743kg;
- Área frontal (Configuração X):0.0197m^2;
- KV do motor: 1400 rpm/V;
- Tensão da bateria: 11.4V;
- Tamanho das hélices: 20x10 cm.

Fonte: [[1]](https://www.dji.com/br/mavic/info)

### Simplificações e Definições Iniciais

O drone é definido como em configuração X, ou seja, a numeração dos motores e sua localização são as seguintes:
- M1: Motor dianteiro esquerdo.
- M2: Motor dianteiro direito.
- M3: Motor traseiro direito.
- M4: Motor traseiro esquerdo.

Isso será importante para a ordem de elementos nas matrizes do Matlab mais adiante.

A estrutura do drone é simplificada como duas hastes em X, fornando 90° entre si. Como já foi dito, o comprimento diagonal é de 335mm, portanto o comprimento lateral será $335mm*sen(45°)=237mm$.

Para efeturar as contas no espaço, é muito importante deixar bem claro os eixos do drone. O **eixo X** é o eixo esquerda/direita do drone, o **eixo Y** é o eixo frente/trás e o **eixo Z** é o eixo cima/baixo. Portanto, para movimentos de **pitch**, será observado o ângulo no eixo X, para **roll** será observado o ângulo do eixo Y e para **Yaw** será observado o ângulo do eixo Z.

![image](https://github.com/user-attachments/assets/3c2761bf-2b1e-44f5-ac28-f5dc6ab2151a)


### 1. Modelagem da planta

Começamos com a modelagem do drone em si. O objetivo aqui é ter como entrada o valor de Thrust, Pitch, Roll e Yaw, e conseguir retornar a aceleração, velocidade e posição linear e angular do drone naquele exato momento. 

Para isso, começamos modelando os motores, calculando a força e torque das hélices para um determinado nível de tensão de entrada. Depois utilizamos esses valores na segunda lei de Newton para calcular a aceleração do corpo. Por fim, são feitas integrais para que a aceleração se torne velocidade e velocidade se torne posição.

#### 1.1 Modelagem dos motores/hélices

No function block **Motor/Hélice**, recebemos como entrada a tensão de alimentação de cada um dos motores, e com isso calcula Torque, Thrust (F) e Corrente em cada motor.

A relação **Tensão-RPM** é encontrada como a velocidade sem carga, utilizando o KV do motor, menos as perdas, que são quadraticamente proporcionais à tensão de entrada, indo até 25% na tensão máxima. [[2]](https://www.udemy.com/course/quadcopter-drone-dji-mavic-matlab-simulink/?couponCode=ST3MT200225A)

Tendo a velocidade dos motores, calculamos a **força** e **torque** gerada nas hélices, calculando o Thrust com a fórmula:

$$
Thrust = C_T * \rho * n^2 * D^4,
$$

Em que $C_T$ é o coeficiente de thrust, $\rho$ é a densidade do ar no nível do mar (1.225kg/m3), $n$ é a quantidade de revoluções por segundo (pode obter com o RPM do motor) e $D$ é o diâmetro da hélice em metros. O *coeficiente de thrust* $C_T$ não é uma constante, mas sim uma função da velocidade. Portanto são tirados os valores do gráfico fornecido pela fabricante para obter a equação de $C_T$ em função do RPM, assim como o torque em função do RPM. [[2]](https://www.udemy.com/course/quadcopter-drone-dji-mavic-matlab-simulink/?couponCode=ST3MT200225A)

Também vemos a relação entre **corrente** e torque:

$$
Torque=K_T*I,
$$

Em que $K_T$ é a constante de torque, que é o inverso da constante de velocidade KV. Ou seja:

$$
I = Torque*KV
$$

```matlab
function [Torque, F, Current] = fcn(Voltage)
    % As variáveis são de tamanho 4 porque representam cada uma das hélices
    RPM = [0 0 0 0];
    Torque = [0 0 0 0];
    Current = [0 0 0 0];
    F = [0 0 0 0];  
    for i=1:4
        RPM(i)=-2.6931*Voltage(i)^3+1400*Voltage(i);
        % Calculando coeficiente de Thrust
        Ct=2*10^-15*RPM(i)^3-4*10^-11*RPM(i)^2+3*10^-7*RPM(i)+0.1013;
        % Dividimos por 60 para dar rotações por segundo
        F(i)=Ct*1.225*(RPM(i)/60)^2*0.2^4;
        Torque(i)=4*10^-14*RPM(i)^3+8*10^-12*RPM(i)^2+3*10^-6*RPM(i);
        Current(i)=1400*Torque(i);
    end
```

#### 1.2 Modelagem da dinâmica angular

Agora fazemos o bloco de dinâmica angular, em que entramos com o torque e força de cada hélice, e retorna a aceleração angular. É utilizada a segunda lei de Newton para rotação, que é definida como:

$$
\begin{align}
M_x=I_x*\theta''_x \\
M_y=I_y*\theta''_y\\
M_z=I_z*\theta''_z
\end{align}
$$

Em que $I$ é o momento de inércia, $M$ é o momento de força (torque) e $\theta''$ é a aceleração angular. Tendo as dimensões do drone, são calculados os momentos de inércia em cada eixo, , tendo como centro de rotação o centro do drone.

$$
\begin{align}
I_{x,y}&=\frac{1}{3}ML^2=\frac{1}{3}\frac{0,743}{4}(\frac{0,237}{2})^2. 4 = 0,003kgm^2 \\
I_z&=\frac{1}{3}\frac{0,743}{4}(\frac{0,335}{2})^2 . 4 = 0,007kgm^2
\end{align}
$$

Para calcular o torque em cada eixo, são utilizadas as forças das hélices, e para o eixo Z, são considerados os torques de cada hélice.

Para que, ao manter Thrust constante e o restante dos comandos zerados, o drone não rode, é necessário que hélices vizinhas girem em sentidos contrários, para que seus torques se cancelem. Por isso as hélices 1 e 3 giram para um lado e 2 e 4 giram para o outro.

```matlab
function RotAcceleration = fcn(Torque, F)
    Moment=[0 0 0];
    RotAcceleration=[0 0 0];
    I=[0.003 0.003 0.007];
   
    Moment(1)= (F(3)+F(4))*0.237/2-(F(1)+F(2))*0.237/2;
    Moment(2)= (F(3)+F(2))*0.237/2-(F(1)+F(4))*0.237/2;
    Moment(3)= Torque(4)-Torque(1)+Torque(2)-Torque(3);
    for i=1:3
        RotAcceleration(i)=Moment(i)/I(i);
    end
```

Tendo a aceleração nos 3 eixos, calcula velocidade e posição angulares através do bloco de integração.

#### 1.3 Modelagem da dinâmica linear

Seguimos a mesma lógica da dinâmica angular, utilizando a segunda lei de Newton, mas agora como é linear, fica:

$$
\begin{align}
F_x=ma_x\\
F_y=ma_y\\
F_z=ma_z
\end{align}
$$

Primeiro calcula as forças das hélices em relação ao **referencial do drone**, para cada eixo XYZ. Depois passa para o referencial do mundo, utilizando os ângulos já calculados de pitch roll e yaw. Para facilitar a visualização das contas:

![image](https://github.com/user-attachments/assets/8bc6c8ae-6d71-49b3-891a-02f2fa5c7cb1)


```matlab
function Acceleration = fcn(F, Disturbance, Theta, Velocity)
    m=0.743;
    g=9.81;
    A=[0.0197 0.0197 0.0512];
    rho=1.225;
    Cd=1;
    
    Acceleration=[0 0 0];
    Force=[0 0 0];
    Fprop=[0 0 0];
    
    % Calculamos as forças em relação ao referencial do drone
    Fprop(1)=sin(Theta(2))*cos(Theta(1))*(F(1)+F(2)+F(3)+F(4));
    Fprop(2)=sin(Theta(1))*cos(Theta(2))*(F(1)+F(2)+F(3)+F(4));
    Fprop(3)=cos(Theta(1))*cos(Theta(2))*(F(1)+F(2)+F(3)+F(4));
```

Para passar para o referencial do mundo, é necessário levar em conta o *Yaw*, que faz com que os eixos XY estejam desalinhados entre os referenciais. É calculado o vetor XY e o ângulo ThetaXY relacionado ao resultante das forças Fpropx e Fpropy no referencial do drone, e soma/subtrai o angulo de yaw, que é o Theta(3). Assim conseguimos ter as novas forças X e Y, no referencial do mundo. O desenho abaixo ajuda a entender os referenciais:

![image](https://github.com/user-attachments/assets/4eb70463-ded7-4c8f-b288-5ba6ac1f5219)




```matlab
    % Depois passamos para o referencial do mundo
    ThetaXY=-atan2(Fprop(1),Fprop(2));
    XY2D=sqrt(Fprop(1)^2+Fprop(2)^2);
    if Fprop(3)>=0
        Fprop(1)=XY2D*sin(ThetaXY+Theta(3));
        Fprop(2)=XY2D*cos(ThetaXY+Theta(3));
    else
        % Quando está indo para baixo
        Fprop(1)=XY2D*sin(ThetaXY-Theta(3));
        Fprop(2)=XY2D*cos(ThetaXY-Theta(3));
    end
```

Por fim, é considerada a resistência do ar e a força do vento, para depois calcular a aceleração nos 3 eixos.
```
    % Drag sempre vai ser contrário ao movimento
    if Velocity(1)<0
        Force(1)=Fprop(1)-Disturbance(1)+0.5*rho*Velocity(1)^2*A(1)*Cd;
    else
        Force(1)=Fprop(1)-Disturbance(1)-0.5*rho*Velocity(1)^2*A(1)*Cd;
    end
    if Velocity(2)<0
        Force(2)=Fprop(2)-Disturbance(2)+0.5*rho*Velocity(2)^2*A(2)*Cd;
    else
        Force(2)=Fprop(2)-Disturbance(2)-0.5*rho*Velocity(2)^2*A(2)*Cd;
    end
    % Para o eixo z, precisa levar em conta também a gravidade
    if Velocity(3)<0
        Force(3)=Fprop(3)-Disturbance(3)+0.5*rho*Velocity(3)^2*A(3)*Cd-m*g;
    else
        Force(3)=Fprop(3)-Disturbance(3)-0.5*rho*Velocity(3)^2*A(3)*Cd-m*g;
    end
    
    for i=1:3
        Acceleration(i)=Force(i)/m;
    end
```

A força do vento é calculada com um bloco separado, seguindo a fórmula:

$$
Drag = \frac{1}{2}\rho V_{vento}^2A C_d,
$$

em que $A$ é a área de superfície, $C_d$ é o coeficiente de arrasto e $\rho$ é a densidade do ar (no nível do mar: $\rho = 1.225kgm^{-3}$). A área no eixo Z é o dobro do resto (duas hastes) e mais 30% por causa das hélices.

```matlab
function DragDisturbance = fcn(VelDisturbance)
    % Área superior é maior por serem 2 hastes e mais as hélices
    A = [0.0197 0.0197 0.0197*2*1.3];
    DragDisturbance=[0 0 0];
    rho=1.225;
    Cd=1;
    
    for i=1:3
        if VelDisturbance(i)<0
            DragDisturbance(i)=0.5*rho*VelDisturbance(i)^2*A(i)*Cd;
        else
            DragDisturbance(i)=-0.5*rho*VelDisturbance(i)^2*A(i)*Cd;
        end
    end
```

Por fim, já com a aceleração linear, a velocidade e posição lineares podem ser calculadas, novamente utilizando o bloco de integração.

#### 1.4 Mixagem dos motores

Tendo modelado a planta inteira, o último passo para seu funcionamento é transformar a entrada de tensão dos 4 motores em um comando mais fácil de controlar, que são Thrust, Pitch, Roll e Yaw. Geralmente ESCs (controlador de motor brushless) comerciais só giram em um sentido, então para manter o realismo, só serão aceitos valores de tensão positiva (0-11,4V). 

Com essa limitação, é necessário definir qual será o intervalo de entrada dos 4 comandos para que a saída seja apenas positiva. O comando de **thrust** aumenta a rotação dos 4 motores ao mesmo tempo, **pitch** deve aumentar a rotação dos motores de trás e diminuir os da frente, **roll** aumenta os motores de um lado e diminui do outro, e para **yaw**, os motores cruzados devem rodar menos que os outros, para que o somatório de torque não dê zero. Dessa forma, a equação para cada motor será:

$$
\begin{align}
Motor_{front left}= Thrust-Pitch+Roll-Yaw \\
Motor_{front right}= Thrust-Pitch-Roll+Yaw \\
Motor_{back right}= Thrust+Pitch-Roll-Yaw \\
Motor_{back left}= Thrust+Pitch+Roll+Yaw
\end{align}
$$

Não precisamos de thrust negativos, visto que a própria gravidade já auxilia, mas para o restante dos comandos, é necessário ter a entrada negativa para que ele possa ir tanto para frente quanto para trás, ou esquerda e direita. Portanto como entrada, os comandos serão limitados assim:
- Thrust: [0,100%];
- Pitch: [-100,100%];
- Roll: [-100,100%];
- Yaw: [-100,100%];

Agora, como podemos garantir que as equações de cada motor não ultrapassem os limites de tensão do motor? 

Será definida uma ordem de **prioridade**. Primeiro é feito o thrust, simplesmente somando seu valor em cada motor. Depois disso, o resto que sobrar de 100% dos motores será utilizado pelo pitch, de forma que o valor de thrust será a média na qual o pitch irá trabalhar em cima. Depois a mesma lógica segue para roll e depois para yaw.

Por **exemplo**, se tenho 60% de pitch, eu ainda tenho 40% para usar de pitch, portanto se eu mandar 100% de pitch, metade dos motores vai estar em 100% (60+40) e a outra metade vai estar em 20% (60-40) para que tenha thrust e pitch ao mesmo tempo, e esse valor não ultrapasse nem 0% nem 100%.

Agora, se eu tenho 40% de thrust, eu estarei mais perto de 0% do que de 100%, fazendo com que ao ter 100% de pitch, metade dos motores vâo para 100% e a outra metade para -20%. Mas como isso não pode acontecer, o valor que sobra para o próximo comando, nesse caso o pitch, terá que ver qual o valor mais próximo (0% ou 100%), para saber quanto o comando pode usar para não ultrapassar os limites.

Agora tem mais um caso. E se o thrust for 0? Nesse caso o resto será 100% e apenas metade dos motores seriam ligados, visto que não tem como o motor girar para o outro lado.

Sabendo tudo isso, o código é escrito, e a saída é normalizada, para passar de 0-100 para 0-11,4.

```matlab
function Voltage = fcn(Thrust, Pitch, Roll, Yaw)

% Limitamos o Thrust apenas para 0-100, porque só gira para um lado
Thrust = max(0,Thrust);

% O resto, depois de aplicar o Thrust, é usado pelo pitch. 
Resto = 100-Thrust;
% Devemos pegar o menor valor em volta do Thrust, então se o resto for
% maior que o thrust, pegamos o próprio thrust
if (Resto > Thrust)&&(Thrust~=0)
    Resto = Thrust;
end
% Agora o 100% do Pitch é equivalente ao resto
Pitch = Pitch*Resto/100;

% O resto, depois de aplicar pitch, é usado pelo roll.
Resto = 100-Thrust-Pitch;
if (Resto > (Thrust+Pitch))&&Thrust~=0&&Pitch~=0
    Resto = Thrust+Pitch;
end
Roll = Roll*Resto/100;
% O resto, depois de aplicar roll, é usado pelo yaw
Resto = 100-Thrust-Pitch-Roll;
if (Resto > (Thrust+Pitch+Roll))&&Thrust~=0&&Pitch~=0&&Roll~=0 
    Resto = Thrust+Pitch+Roll;
end
Yaw = Yaw*Resto/100;

% Matriz de mixagem
M1 = Thrust - Pitch + Roll - Yaw;
M2 = Thrust - Pitch - Roll + Yaw;
M3 = Thrust + Pitch - Roll - Yaw;
M4 = Thrust + Pitch + Roll + Yaw;

% Limita os valores dos motores entre 0 e 100, e normaliza para a tensão da
% bateria
M1 = (max(0, min(100, M1)))*11.4/100;
M2 = (max(0, min(100, M2)))*11.4/100;
M3 = (max(0, min(100, M3)))*11.4/100;
M4 = (max(0, min(100, M4)))*11.4/100;

Voltage = [M1   % Motor dianteiro esquerdo
           M2   % Motor dianteiro direito
           M3   % Motor traseiro direito
           M4]; % Motor traseiro esquerdo
```

Com isso, o drone pode ser testado, utilizando o bloco de **Joystick Input**, que funciona com qualquer controle de video game que o computador reconheça como de Xbox. Lembrar de utilizar o bloco **Set Pace** junto para que a simulação fique em tempo real.

![image](https://github.com/user-attachments/assets/095f8f2d-faf2-4ae3-a223-b16332f7fe06)



### 2. Projeto de controle do drone

Para o controle de posição do drone, são feitas duas malhas de controle: (1) interna, controlando o ângulo e (2) externa, controlando a posição.

#### 2.1 Controle de ângulo

No controle de ângulo, apenas os comandos de pitch, roll e yaw são alterados. É feita a realimentação do ângulo do drone para um bloco somador, para que seja calculado o erro com o comando de referência. O ângulo do eixo X é relacionado com o comando de Pitch e o ângulo do eixo Y é relacionado com o Roll.

Nessa etapa, foi definido que o **ângulo do drone é limitado** para o intervalo [-30°,+30°], garantindo que ele não ultrapasse esse valor e acabe ficando de ponta-cabeça. O referencial foi limitado utilizando bloco de saturação na entrada do somatório. Como esse referencial posteriormente vai vir do controle de posição, a saturação evita que, ao estar em distâncias muito grandes do destino, o drone tenha risco de tombar.

Como o comando de yaw não tem risco de deixar o drone de ponta-cabeça, essa limitação não foi necessária nesse eixo.

O próximo passo foi entrar com esse erro no **controlador PID**. Seus coeficientes foram sintonizados manualmente, porque o tuner não conseguiu linearizar a planta.

Cada comando foi sintonizado individualmente, enquanto os outros ficavam zerados. Começando apenas com $K_p=1$, foram encontrados os valores finais, e o PID de Pitch e Roll foram iguais, visto que a planta se comporta igual nos dois eixos. No comando de 

Todos os PIDs tiveram a limitação de saída habilitada, para o intervalo [-100,100], como explicado na mixagem.

![image](https://github.com/user-attachments/assets/2c0ecc04-7c8c-45cb-9b17-6e202f509eea)



#### 2.2 Controle de posição

No controle de posição, apenas os comandos de thrust, pitch e roll são alterados. É feita a realimentação da posição do drone para um bloco somador, para que seja calculado o erro com o comando de referência. A posição em Y é relacionado com o comando de Pitch e a posição em X é relacionado com o Roll.

Os **controladores PID** foram sintonizados da mesma forma que para o ângulo. Seus coeficientes foram sintonizados manualmente, porque o tuner não conseguiu linearizar a planta.

Como já tinha a saturação de 30° na saída do PID, a limitação interna deles não foi habilitada.

![image](https://github.com/user-attachments/assets/73b08624-5c1c-46d0-bb3c-4021f09bbbf1)



### Resultados

Com o gráfico abaixo é possível observar que o drone conseguiu chegar nos valores desejados de posição (definidos no canto esquerdo), não ultrapassando os valores de ângulo definidos, fazendo uma trajetória de segurança.

![image](https://github.com/user-attachments/assets/2a8ccaea-3e0b-4126-a465-c94aa2ba1237)


Em mais um teste, é possível ver que o controle funciona, mesmo com **perturbações externas**, que nesse caso são ventos de 10m/s na direção X.

![image](https://github.com/user-attachments/assets/e39bd8eb-3563-4243-b886-0ba07bdd24c5)


A trajetória do drone pode ser visualizada com o bloco sim.out, utilizando o seguinte comando no prompt de comando:

```matlab
PositionVector = out.PositionVector;
plot3(PositionVector(:,1),PositionVector(:,2),PositionVector(:,3),'-r')
xlabel('X')
ylabel('Y')
zlabel('Z')
```

A visualização pode ser bem melhor com o `Aerospace Blockset`, mas eu tentei rodar nesse computador, que tem Matlab 2020, e ficou carregando infinitamente. Mas tem [esse](https://youtu.be/FUitbiQ2XG4?si=OmZCYEriz8rV9NxM) vídeo que explica como fazer.

# Referências

[[1]](https://www.dji.com/br/mavic/info) **Especificações do drone DJI Mavic Pro**;

[[2]](https://www.udemy.com/course/quadcopter-drone-dji-mavic-matlab-simulink/?couponCode=ST3MT200225A) Curso na Udemy **Model, Simulate and Control a Drone in MATLAB & SIMULINK**;

[[3]](https://youtube.com/playlist?list=PLn8PRpmsu08oOLBVYYIwwN_nvuyUqEjrj&si=Xy4R3s5VU87XezLR) Playlist do Matlab **Drone Simulation and Control**;

[[4]](https://youtu.be/yIScfmQAEGY?si=vJbgxWAOFtwy-bWS) **How to create a mathematical model of a Quadcopter**, por ANTSHIV ROBOTICS;

[[5]](https://youtu.be/iS5JFuopQsA?si=XuZt2W4X1TaT-tor) **MATLAB & Simulink Tutorial: Quadrotor UAV Trajectory and Control Design (PID + Cascaded)**, por Vinayak D.

