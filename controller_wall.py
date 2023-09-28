# Código para controle de robô Epuck (WeBots)

# Importando a classe de robô
from controller import Robot

# Definindo o robô que segue a parede
def run_robot(robot):
    
    # Definindo o passo 
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.28

    # Habilitando os motores do robô
    left_motor = robot.getMotor('left wheel motor')
    right_motor = robot.getMotor('right wheel motor')

    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)

    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

    # Habilitando os sendores de proximidade
    prox_sensors = []
    for ind in range(8):
        sensor_name = 'ps' + str(ind)
        prox_sensors.append(robot.getDistanceSensor(sensor_name))
        prox_sensors[ind].enable(timestep)

    # Loop Principal (ATENÇÃO!!!)
    while robot.step(timestep) != -1:
        # Lendo os sensores
        for ind in range(8):
            print("ind: {}, val: {}".format(ind, prox_sensors[ind].getValue()))

        # Processamento dos dados do sensor
        # IMPORTANTE!!!!

        left_wall = prox_sensors[5].getValue() > 80 # Sensor 5 não detecta parede
        front_wall = prox_sensors[7].getValue() > 80 # Sensor 7 não detecta parede
        left_corner = prox_sensors[6].getValue() > 80 # Sensor 6, que detecta a "esquina"

        # Orientação das rodas
        left_speed = max_speed
        right_speed = max_speed

        # Caso 1: Parede à frente!
        if front_wall:
            print ("Virando para a direita")
            left_speed = max_speed
            right_speed = -max_speed
        else:
            if left_wall:
                print ("Segue em frente companheiro...")
                left_speed = max_speed
                right_speed = max_speed
            else:
                print ("Virar para a esquerda")
                left_speed = max_speed/8
                right_speed = max_speed

            if left_corner:
                print("Muito perto da quina")
                left_speed = max_speed
                right_speed = max_speed/8

        # Envio de dados para o ATUADOR    
        left_motor.setVelocity(max_speed)
        right_motor.setVelocity(max_speed)

if __name__ == "__main__":
    # Criação da Instância do robô
    my_robot = Robot()
    run_robot(my_robot)