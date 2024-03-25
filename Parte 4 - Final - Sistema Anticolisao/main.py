# fabio martins
# 20 out 2023 
# https://github.com/fdmartins/RoboCarRace

from machine import Pin, PWM, I2C
import utime
import time
import _thread


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
        
class Ultrassom:

    def __init__(self,  trigger_pin, echo_pin):
        self.trigger = Pin(trigger_pin, Pin.OUT) #GPIO
        self.echo = Pin(echo_pin, Pin.IN) 
        self.distance = 0

    def get_distance(self):
        # retorna ultima lida.
        return self.distance

    def read(self):
        # lemos a distancia.

        # acionamos o trigger, precisa ser de 10us, mas o tempo da funcao em micropython eh lenta, por isso consideramos 2.
        # https://www.robot-electronics.co.uk/htm/srf05tech.htm
        utime.sleep_us(2)
        self.trigger.high()
        utime.sleep_us(2)
        self.trigger.low()

        
        signaloff = signalon = 0
        start = utime.ticks_us()
        timeout = False
        
        while self.echo.value() == 0:
            signaloff = utime.ticks_us()
            if (start - signaloff)>40000:
                #timeout - limite para nao ficar preso no looping.
                timeout = True
                break
            
        while self.echo.value() == 1:
            signalon = utime.ticks_us()
            if (signalon - signaloff)>40000:
                #timeout - limite para nao ficar preso no looping.
                timeout = True
                break

        timepassed = signalon - signaloff
        
        # distancia em cm
        # velocidade do som 340 m/s
        distance = (timepassed * 0.0340) / 2.0 
        

        if timeout:
            distance = 0    
        
        #print(timeout, timepassed, distance)
        
        
        if distance<5 or distance>500:
            # erros medicao.
            self.distance = 0
            
        
        self.distance = distance


class ReadPWM:
    def __init__(self, pin, min_value, max_value ):
        self.pin = Pin(pin, Pin.IN)  #GPIO
        self.min_value = min_value
        self.max_value = max_value

        # tempo minimo 1000us
        # tempo maximo 2000us.
        self.min_duty = 1000
        self.max_duty = 2000
        self.__conversion_factor =  (self.max_value - self.min_value) / (self.max_duty - self.min_duty)

    def read(self):
        # espera sinal cair.
        while self.pin.value() == 1: 
            pass

        # espera sinal levantar.
        while self.pin.value() == 0: 
            pass

        start = utime.ticks_us()

        # espera sinal cair.
        while self.pin.value() == 1: 
            pass
        
        end = utime.ticks_us()

        timepassed = end - start
        

        return int((timepassed - self.min_duty) * self.__conversion_factor) + self.min_value

###########################
######## INICIAMOS ########
###########################

led_onboard = machine.Pin(25, machine.Pin.OUT)

motor_tracao = Servo(
        min_angle=-100, #  correspondendo a 100% da velocidade de ré
        max_angle=100, #  correspondendo a 100% da velocidade de frente.
        pin=9  # ajuste com o pino GPIO que esta ligado o sinal do ESC
    )  


servo_dir = Servo(
        min_angle=-45, 
        max_angle=45, # utilizamos um servo de 90 graus. Ajuste conforme o que vc usa.
        pin=6 
    )  

ultrassom_left = Ultrassom(
    trigger_pin=0,
    echo_pin=1
)

ultrassom_center = Ultrassom(
    trigger_pin=2,
    echo_pin=3
)

ultrassom_right = Ultrassom(
    trigger_pin=4,
    echo_pin=5
)

pwm_input_servo_dir = ReadPWM(
    pin=21,
    min_value = -45,
    max_value = 45
)

pwm_input_motor_tracao = ReadPWM(
    pin=20,
    min_value = -100,
    max_value = 100
)


#I2C_ADDRS=0x18
#i2c = I2C(
#    I2C_ADDRS, 
#    scl=Pin(27), 
#    sda=Pin(26), 
#    freq=100000)


##################################################################
######        LOOPING VERIFICACAO DISTANCIA SEGURANCA     ########
##################################################################

def task_ultrasound():
    while True:
        ultrassom_left.read()
        ultrassom_center.read()
        ultrassom_right.read()
        time.sleep(1.05)

# iniciamos a thread.
# https://bytesnbits.co.uk/multi-thread-coding-on-the-raspberry-pi-pico-in-micropython/
_thread.start_new_thread(task_ultrasound, ())

#################################
###### LOOPING PRINCIPAL ########
#################################

# ao ligar, o ESC precisa ser iniciado.
# para isso colocamos a velocidade em NEUTRO.
motor_tracao.move(0)
# aguardamos o ESC iniciar.
time.sleep(5)

print("INICIADO")

while True:
    
    if False:
        # Verificamos placa principal enviou alguma mudanca.
        # TODO
        pass
    else:
        # ou se modo treinamento, lemos do receptor/pwms
        pass

    speed = 10
    steerwheel = 0

    # verificamos se estamos com delay na recepcao dos dados.
    # TODO.

    # verificamos se existe colisao.
    
    if ultrassom_left.get_distance() < 100000:
        speed = 5
        steerwheel = 45
        #print("SENSOR COLISAO ESQUERDO: " ,  ultrassom_left.get_distance())

    if ultrassom_right.get_distance() < 100000:
        speed = 5
        steerwheel = -45
        #print("SENSOR COLISAO DIREITO: " , ultrassom_right.get_distance())

        
    if ultrassom_center.get_distance() < 100000:
        speed = 0
        #print("SENSOR COLISAO CENTRAL: " , ultrassom_center.get_distance())

    # ajustamos direcao.
    servo_dir.move(steerwheel)
    
    # ajustamos velocidade.
    motor_tracao.move(speed)

    led_onboard.toggle()
    time.sleep(0.1)