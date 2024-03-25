# -*- coding: iso-8859-1 -*-

# Fabio Martins - interapix.com.br
# 03/11/2023
 
# PARA AUTO START VER: 
# https://askubuntu.com/questions/133235/how-do-i-allow-non-root-access-to-ttyusb0
# https://forums.developer.nvidia.com/t/jetson-nano-auto-run-python-code-when-power-up/108999
# https://unix.stackexchange.com/questions/225401/how-to-see-full-log-from-systemctl-status-service


import enum
#import smbus # apt-get install python-smbus
import time 
from threading import Thread 
import serial
import csv

import cv2

import time
import threading
import datetime 
import numpy as np

#import imp
#import inputs 
import threading

import Adafruit_SSD1306   # pip3 install Adafruit-SSD1306
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont


class DisplaySSD1306:
    def __init__(self):
        # 128x32 display with hardware I2C:
        # setting gpio to 1 is hack to avoid platform detection
        self.disp = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=1, gpio=1)

        # Initialize library.
        self.disp.begin()

        # Clear display.
        self.disp.clear()
        self.disp.display()

        #Load default font.
        self.font = ImageFont.load_default()
   
        # Create blank image for drawing.
        # Make sure to create image with mode '1' for 1-bit color.
        self.width = self.disp.width
        self.height = self.disp.height
        self.image = Image.new('1', (self.width, self.height))

        # Get drawing object to draw on image.
        self.draw = ImageDraw.Draw(self.image)

        # Draw a black filled box to clear the image.
        self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)

        self.draw.text((0,0), "Aguardando Pico...", font=self.font, fill=255)

        self.disp.image(self.image)
        self.disp.display()

    def write(self, x,y, text, clear=True):

        if clear:
            # Draw a black filled box to clear the image.
            self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)

        self.draw.text((x, y), text, font=self.font, fill=255)

        # Display image.
        # Set the SSD1306 image to the PIL image we have made, then dispaly
        self.disp.image(self.image)
        self.disp.display()

class CarComm:
    def __init__(self, serialPort, baudrate = 115200):
        self.running = True
        self.serial = serial.Serial(
            port = serialPort, 
            baudrate = baudrate, stopbits=1, timeout=100)  
              
        self.thread = threading.Thread(target = self.update)

        self.mode = 0
        self.speed = 0
        self.steer = 0
        self.sonar_r = 0
        self.sonar_c = 0
        self.sonar_l = 0

        self.last_comm = datetime.datetime.now()

        self.data_length_tx = 2
        self.data_length_rx = 6
        
        self.start_byte = "*"
        self.end_byte = "\n"

        self.thread.start()
        
    def stop(self):
        self.running = False

    def get_speed(self):
        return self.speed
    def get_steer(self):
        return self.steer
    def set_speed(self, v):
        self.speed = v
    def set_steer(self,v):
        self.steer = v


    def isOnline(self):
        if (datetime.datetime.now()-self.last_comm).total_seconds()<2:
            return True
        return False

    def update(self):

        data = self.serial.read(self.serial.inWaiting())
        print("garbage collection:", len(data))

        while self.running:

            #if(self.speed>100):
            #    self.speed=-100
            #self.speed+=1
            #self.steer=100-self.speed
            
            try:
                # enviamos a velocidade e direcao.
                package = [
                            
                            str(self.speed),
                            str(self.steer),
                           
                        ]

                data_tx = self.start_byte + ('|'.join(package)) +  self.end_byte

                data_tx = data_tx.encode()
                #print(data_tx)
                self.serial.write(data_tx)
                
                #print("enviado >>>> ", data_tx)
                
                while(self.serial.inWaiting()<13):
                    # TODO TIMEOUT
                    pass
                
                
                # lemos todos os bytes no buffer da serial...
                data = self.serial.readline()

                #print("recebido <<< ", data)
                
                if len(data)<3:
                    continue

                data = data.decode("UTF-8")


                if data[0]==self.start_byte and data[-1]==self.end_byte:
                    info = data[1:-1].split('|')
                    #print(info)
                    if len(info)==self.data_length_rx:
                        #print(datetime.datetime.now(), info)
                        self.mode = int(info[0])
                        self.speed = int(info[1])
                        self.steer = int(info[2])
                        self.sonar_l = int(info[3])
                        self.sonar_c = int(info[4])
                        self.sonar_r = int(info[5])
                        
                self.last_comm = datetime.datetime.now()

                time.sleep(0.01)


            except Exception as err:
                print("Erro")
                print(err)


class Camera:
    def __init__(self):
        print(self.gstreamer_pipeline(flip_method=0))
        self.cap = cv2.VideoCapture(self.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

        if self.cap.isOpened():
            ret_val, img = self.cap.read()
            if ret_val==False:
                print("Sem Camera Encontrada") 
                exit()
        else:
            print("Sem Camera Encontrada")  
            exit()

    def gstreamer_pipeline(self,
        capture_width=1280,
        capture_height=720,
        display_width=640,
        display_height=360,
        framerate=120,
        flip_method=0,
        ):
        return (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink drop=1"
            % (
                capture_width,
                capture_height,
                framerate,
                flip_method,
                display_width,
                display_height,
            )
        )

    def get_frame(self):
        if self.cap.isOpened():
            ret_val, img = self.cap.read()
            if ret_val==False:
                print("Falha Camera") 
                exit()

            return img
        else:
            print("Sem Camera Encontrada")  


class RecordData:
    def __init__(self):
        self.running = True              
        self.buffer = []
        self.recording = False
        self.thread = threading.Thread(target = self.async_save)

        self.thread.start()
        

    def startFiles(self, file_name):
        self.frame_width, self.frame_height = (640,360)
        fps = 60
        self.video_file = cv2.VideoWriter(file_name + '.avi',cv2.VideoWriter_fourcc('M','J','P','G'), fps, (self.frame_width,self.frame_height))

        self.csv_file = open(file_name + '.csv', 'w', newline='') 
        self.csv_writer = csv.writer(self.csv_file)

        print(file_name)

        self.buffer = []
        self.recording = True

    def async_save(self):
        while(self.running):
            while(len(self.buffer)>0):
                #print(len(self.buffer))
                data, frame = self.buffer.pop(0)

                self.video_file.write(frame)
                self.csv_writer.writerow(data)
            time.sleep(0.05)


    def closeFiles(self):
        while(len(self.buffer)>0):
            print("Aguardando salvar dados...", len(self.buffer))
            time.sleep(0.5)

        self.recording = False

        # finaliza arquivo de video...
        self.video_file.release()

        # fecha arquivo csv.
        self.csv_file.close()

    def checkBuffer(self):
        print("acumulado Buffer:", len(self.buffer))

    def write(self, data, frame):
        self.buffer.append([data, frame])
        

# podemos usar um adaptador serial, ou a porta do jetson/rasp.
serialPort = "/dev/ttyUSB0"

display = DisplaySSD1306()

camera = Camera()

chassis = CarComm(serialPort)

record_data = RecordData()

last_mode = None
idx_learning = 0
fps = 0
start_time = datetime.datetime.now()
waiting_speed = True

while True:
    if chassis.mode != last_mode:
        if chassis.mode == 1:
            print("Manual")
            display.write(0,0, "Manual")
            
        if chassis.mode == 2:
            print("Semi Auto")
            display.write(0,0, "Semi Auto")
            
        if chassis.mode == 3:
            print("Full Drive")
            display.write(0,0, "Full Drive")
            
        if chassis.mode == 4:
            print("Learning Mode")
            display.write(0,0, "Learning Mode")
            

            # inicia arquivo de video...
            file_name = "train_{}".format(datetime.datetime.now())

            # inicia arquivo csv...
            record_data.startFiles(file_name)

            idx_learning = 0

        if last_mode==4 and chassis.mode!=4:
            print("Salvando Treino...")
            
            record_data.closeFiles()
            
            print("Feito")

        last_mode = chassis.mode

    if chassis.isOnline()==False:
        print("timeout rasp")
        display.write(0,0, "Timeout Rasp!!")
        time.sleep(1)

    if chassis.mode==4:
        # aprendendo...
        if chassis.speed>5:
            idx_learning+=1
            if waiting_speed:
                display.write(0,0, "Registrando...")
                waiting_speed=False

            # captura frame
            frame = camera.get_frame()
            
            # registra dado csv.
            data = [idx_learning, chassis.mode, chassis.speed, chassis.steer, chassis.sonar_r, chassis.sonar_c, chassis.sonar_l]
            

            # gravamos cada 2 frames, para nao acumular muito no buffer de escrita no arquivo.
            # a captura com a camera conseguimos taxas de 60fps. Deste modo estamos coletando dados a cada 30fps.
            if idx_learning%2==0:
                # Sem latencia. Apenas inserimos em uma fila. A escrita no arquivo e assincrona em uma thread independente.
                record_data.write(data, frame)
        
            # print(csv_line)
            
            
            fps+=1
            if (datetime.datetime.now()-start_time).total_seconds()>=1:
                print("FPS:{}".format(fps))
                start_time = datetime.datetime.now()
                fps = 0
                record_data.checkBuffer()
        else:
            display.write(0,0, "Acelere! idx:{}".format(idx_learning))
            waiting_speed = True

    time.sleep(0.001)

 