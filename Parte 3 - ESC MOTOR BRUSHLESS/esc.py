# fabio martins
# 16 out 2023 
# https://github.com/fdmartins/RoboCarRace

from machine import Pin, PWM

class Servo:
    __motor = None
    __servo_pwm_freq = 50
    __min_u16_duty = 3300  # 1ms min cycle 
    __max_u16_duty = 6600  # 2ms max cycle
    min_angle = 0
    max_angle = 180 
    current_angle = 0.001


    def __init__(self,  min_angle, max_angle, pin):
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.__initialise(pin)


    def move(self, angle):
        # Arredondamos para comparacao
        angle = round(angle, 2)
        # precisa mover?
        if angle == self.current_angle:
            return
        self.current_angle = angle
        # calculamos novo duty cicle.
        duty_u16 = self.__angle_to_u16_duty(angle)
        self.__motor.duty_u16(duty_u16)

    def __angle_to_u16_duty(self, angle):
        # esta funcao calculo o duty cicle proporcional em relacao aos angulos minimo a maximo e os dutys min e max.
        return int((angle - self.min_angle) * self.__angle_conversion_factor) + self.__min_u16_duty


    def __initialise(self, pin):
        self.current_angle = -0.001
        self.__angle_conversion_factor = (self.__max_u16_duty - self.__min_u16_duty) / (self.max_angle - self.min_angle)
        self.__motor = PWM(Pin(pin))
        self.__motor.freq(self.__servo_pwm_freq)
        
        
import time

servo = Servo(
        min_angle=-100, #  correspondendo a 100% da velocidade de ré
        max_angle=100, #  correspondendo a 100% da velocidade de frente.
        pin=9  # ajuste com o pino GPIO que esta ligado o sinal do ESC
    )  



# ao ligar, o ESC precisa ser iniciado.
# para isso colocamos a velocidade em NEUTRO.
# lembrar que:
# <0 para dar ré
# >0 para ir para frente
servo.move(0)

while True:
    # andamos 1 segundo para frente. Com 20% da velocidade maxima
    servo.move(20)
    time.sleep(1)

    # andamos 1 segundo de ré. Com 10% da velocidade maxima
    servo.move(10) 
    time.sleep(1)

    # repete while