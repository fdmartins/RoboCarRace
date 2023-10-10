from machine import Pin, Timer
import utime



trigger = Pin(0, Pin.OUT) #GPIO0
echo = Pin(1, Pin.IN) #GPIO1


def ultrassound():
   #trigger.low()
   utime.sleep_us(2)
   trigger.high()
   utime.sleep_us(2)
   trigger.low()
 
   signaloff = signalon = 0
   start = utime.ticks_us()
   while echo.value() == 0:
       signaloff = utime.ticks_us()
       if (start - signaloff)>20000:
           #timeout - limite para nao ficar preso no looping.
           break
       
   while echo.value() == 1:
       signalon = utime.ticks_us()
       if (signalon - signaloff)>20000:
           #timeout - limite para nao ficar preso no looping.
           break

   timepassed = signalon - signaloff
   distance = timepassed
   distance = (timepassed * 0.0340) / 2.0 # velocidade do som 340 m/s
 
   if distance<5 or distance>500:
       # erros medicao.
       return None
    
   
   return distance
   

while True:

    for i in range(0,5):
        # 3 tentativas em caso de erro.
        distance = ultrassound()
    
        if distance!=None:
            distance = "{:.1f}".format(distance)
            break
        else:
            # erro - do nothing, tentamos novamente.
            pass
        
    print(distance)

    # leitura a cada 500 ms.
    utime.sleep(0.5)