from controller import Robot, Motor, Keyboard
from pathfinding import Pathfind

robot = Robot()

timestep = int(robot.getBasicTimeStep())

keyboard = Keyboard()
keyboard.enable(timestep)

# ----- Inicializa os motores -----
motor_l = robot.getMotor('left wheel motor')
motor_r = robot.getMotor('right wheel motor')
motor_l.setPosition(float('inf'))
motor_r.setPosition(float('inf'))

motor_l.setVelocity(0.0)
motor_r.setVelocity(0.0)


# ----- Funções de controle do robo -----
# Funções criadas de forma empírica
def RobotTurnRight():
    motor_r.setVelocity(-2.197)
    motor_l.setVelocity(2.197)
    robot.step(1000)
    motor_l.setVelocity(0.0)
    robot.step(timestep)

def RobotTurnLeft():
    motor_r.setVelocity(2.097)
    motor_l.setVelocity(-2.097)
    robot.step(1000)
    motor_r.setVelocity(0.0)
    robot.step(100)
    

def RobotGoForward():
    motor_r.setVelocity(4.13)
    motor_l.setVelocity(4.13)
    robot.step(1000)
    motor_l.setVelocity(0.0)
    motor_r.setVelocity(0.0)
    robot.step(100)

def RobotFollowPath(path, start_angle):
    for i in range(len(path)-1):
        motion_x = path[i+1][1] - path[i][1]
        motion_y = path[i+1][0] - path[i][0]
        
        # Descobre o angulo da direção do movimento
        angle = (motion_y != 0)*(motion_y+2)*90 + \
                (motion_x != 0)*abs(motion_x-1)*90

        # Diferença entre o angulo do robo e do movimento
        delta_ang = start_angle - angle
        
        # Conta quantos vezes tem que virar pra esquerda ou direita
        if (delta_ang < 0):
            esq = -delta_ang//90
            dir = (360+delta_ang)//90
        else:
            esq = (360-delta_ang)//90
            dir = delta_ang//90
                            
        # Se tiver calculado uma volta completa, 0 os valores
        if (esq == 4 or dir == 4):
            esq = 0
            dir = 0
            
        # Verifica qual o melhor jeito de virar
        if (esq <= dir):
            for n in range(esq):
                RobotTurnLeft()
        elif (esq > dir):
            for n in range(dir):
                RobotTurnRight()
        
        # Atualiza o angulo do robo
        start_angle = angle
        
        RobotGoForward()
        
    return start_angle
        
        

# ----- Declarações do algoritmo de Pathfinding -----
map = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0],
       [0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0],
       [0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

goal_pos = [1, 9]

start_pos = [7, 7]
start_angle = 90

pathfind = Pathfind()

# Encontra o caminho ao objetivo
path = pathfind.path_plan(start_pos, goal_pos, map)

# Faz o robo seguir o caminho
RobotFollowPath(path, start_angle)
