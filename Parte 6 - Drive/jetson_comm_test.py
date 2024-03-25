# -*- coding: iso-8859-1 -*-

# Fabio Martins - interapix.com.br
# 03/11/2023
 
# PARA AUTO START VER:.. 
# https://askubuntu.com/questions/133235/how-do-i-allow-non-root-access-to-ttyusb0
# https://forums.developer.nvidia.com/t/jetson-nano-auto-run-python-code-when-power-up/108999
# https://unix.stackexchange.com/questions/225401/how-to-see-full-log-from-systemctl-status-service

# VER ERRO AUTO START:
# journalctl -u robocar.service  

# sudo systemctl stop robocar.service

# FALHA CAM:
# sudo service nvargus-daemon restart

# LIBERAR MEMORIA:
# sudo systemctl stop display-manager

import enum
#import smbus # apt-get install python-smbus
import time 
import serial
import csv
import os
import socket

import cv2 # INICIE O OPENCV ANTES DO TENSORFLOW!!

import time
import datetime 
import numpy as np

#import imp
#import inputs 
import threading

# pip3 install Adafruit-SSD1306
import Adafruit_SSD1306  
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont


#  pip3 install https://dl.google.com/coral/python/tflite_runtime-2.1.0.post1-cp36-cp36m-linux_aarch64.whl
import tflite_runtime.interpreter as tflite  

    

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
        #self.font = ImageFont.load_default()
        self.font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14)
   
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


    def write_assync(self, x, y, text, clear=True):
        self.thread = threading.Thread(target = self.write, args=(x, y, text, clear,))
        self.thread.start()
        

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
                        if self.mode == 4:
                            # apenas reescreve se estiver em learning mode
                            self.speed = int(info[1])
                            self.steer = int(info[2])
                        self.sonar_l = int(info[3])
                        self.sonar_c = int(info[4])
                        self.sonar_r = int(info[5])
                else:
                    print("corrompido")
                    print(data)

                self.last_comm = datetime.datetime.now()

                time.sleep(0.01)


            except Exception as err:
                print("Erro")
                print(err)


class Camera:
    def __init__(self):
        print("Criando Camera...")

        cmd = 'sudo service nvargus-daemon restart'
        ret = os.system(cmd)
        print(ret)

        print(self.gstreamer_pipeline(flip_method=0))

        self.cap = cv2.VideoCapture(self.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

        #self.cap = cv2.VideoCapture("./datatrain/train_2023-11-03 18:16:27.587601.avi")

        self.running = True
        self.last_frame = None

        if self.cap.isOpened():
            print("Camera Aberta!")
            ret_val, img = self.cap.read()
            if ret_val==False:
                print("Sem Camera Encontrada") 
                exit()

            print("Camera Read OK!")
            self.thread = threading.Thread(target = self.update)
            self.thread.start()

        else:
            print("Sem Camera Encontrada")  
            exit()


    def gstreamer_pipeline(self,
        capture_width=1280,
        capture_height=720,
        display_width=640,
        display_height=360,
        framerate=60,
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
        return self.last_frame

    def update(self):
        print("Thread Iniciada.")
        #idx = 0
        while self.running:
            ret_val, img = self.cap.read()
            if ret_val==False:
                #print("Falha Camera") 
                self.last_frame = None

            #print("Video:{} {}".format(ret_val, idx))
            #idx+=1
            self.last_frame = img

            # para video usar delay abaixo!
            #time.sleep(0.0026)

class OnlineView:
    def __init__(self):
        # Configurações do servidor
        self.host = '192.168.1.154'  # Endereço IP do servidor
        self.port = 12345         # Porta do servidor

        # Configuração do socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((self.host, self.port))
        self.sock.listen(1)



        self.thread = threading.Thread(target = self.waitConn)

        self.thread.start()
        
        self.last_frame = None

    def put(self, frame):
        self.last_frame = frame

    def waitConn(self):
        while True:
            # Aceita a conexão do cliente
            conn, addr = self.sock.accept()
            print("cliente conectado")
            while True:
                time.sleep(0.01)
                if np.all(self.last_frame)==None:
                    continue

                #print("escrito frame")
                #conn.write(self.last_frame)
                 # Codifique o frame em formato JPEG
                _, img_encoded = cv2.imencode('.jpg', self.last_frame)
                data = np.array(img_encoded).tobytes()

                # Envie o tamanho do frame
                conn.sendall((str(len(data))).encode('utf-8').ljust(16))

                # Envie o frame
                conn.sendall(data)





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
        self.last_record = datetime.datetime.now()

    def async_save(self):
        while(self.running):
            while(len(self.buffer)>0):
                #print(len(self.buffer))
                data, frame = self.buffer.pop(0)

                self.video_file.write(frame)
                self.csv_writer.writerow(data)
            time.sleep(0.05)


    def closeFiles(self):
        print("Salvando Treino...")
        while(len(self.buffer)>0):
            print("Aguardando salvar dados...", len(self.buffer))
            time.sleep(0.5)

        self.recording = False

        # finaliza arquivo de video...
        self.video_file.release()

        # fecha arquivo csv.
        self.csv_file.close()
        print("OK")

    def checkBuffer(self):
        print("acumulado Buffer:", len(self.buffer))

    def write(self, data, frame):
        if self.recording==False:
            return

        # gravamos apenas alguns, para nao acumular muito no buffer de escrita no arquivo (que eh lento.).
        if (datetime.datetime.now()-self.last_record).total_seconds()*1000 > 33:
            self.buffer.append([data, frame])
            self.last_record = datetime.datetime.now()

class FullDrive:


    def __init__(self):
        # Carrega o modelo  treinado
        print("Carregando Modelo...")
        
        # usamos tflite
   
        self.interpreter = tflite.Interpreter(model_path="model.tflite") # 120fps
        #self.interpreter = tflite.Interpreter(model_path="model_attention.tflite")
        #self.interpreter = tflite.Interpreter(model_path="model_D16_dropout.tflite") #110fps

        self.interpreter.allocate_tensors()

        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        print("OK Modelo Carregado!")
        
        self.smooth = []

    def preenchimento(self, img, seed, nova_cor):

        # Define uma máscara para o preenchimento
       # mask = np.zeros_like((202,202), np.uint8)

        # Define o valor do pixel na semente na máscara como 255 (branco)
        #mask[seed[0], seed[1]] = 255

        # Aplica o algoritmo de preenchimento
        cv2.floodFill(img, None, seedPoint=seed, newVal=(255,255,255), loDiff=(12,12,12), upDiff=(12,12,12))
        #cv2.circle(img, seed, 2, (0,255,0), cv2.FILLED, cv2.LINE_AA)
        return img
        
        
    def get_action(self, frame):
        # todo - colocar toda essa parte na thread da camera.
        # dimensoes do frame.
        altura, largura = frame.shape[:2]

        # redimentsionamos(x,y)
        novo_tamanho = (200, 200)
        
        #print("Crop...")
        #frame = frame[int(altura/4):,:]

        # Redimensionar a imagem para o novo tamanho com interpolação cv2.INTER_LINEAR
        #print("Resize...")
        frame = cv2.resize(frame, novo_tamanho)

        #print("cvtColor...")
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)


        # normalizamos os pixels do frame entre 0 e 1.
        #print("frame norm...")
        frame_norm = frame#.astype('float32') #/ 255.

        #return 0,0
        posicao_amostra = (100, 170)


        if False:
            # Crie uma máscara gradiente para definir a região de desfoque gradual
            altura, largura, _ = frame_norm.shape
            mascara = np.zeros((altura, largura), dtype=np.uint8)

            # Defina as coordenadas da região de desfoque gradual
            inicio_desfoque = (0, 150)  # coordenadas iniciais da região
            fim_desfoque = (200, 200)    # coordenadas finais da região
            cv2.rectangle(mascara, inicio_desfoque, fim_desfoque, 255, thickness=cv2.FILLED)

            # Aplique o desfoque à imagem usando a máscara gradiente
            desfoque_gradual = cv2.GaussianBlur(frame_norm, (3, 3), 0)
            frame_norm = np.where(mascara[:, :, None], desfoque_gradual, frame_norm)

        if True:
            # Divida a imagem em canais de cor
            canal_azul, canal_verde, canal_vermelho = cv2.split(frame_norm)

            # Aplique a equalização de histograma a cada canal
            canal_azul_eq = cv2.equalizeHist(canal_azul)
            canal_verde_eq = cv2.equalizeHist(canal_verde)
            canal_vermelho_eq = cv2.equalizeHist(canal_vermelho)

            # Junte os canais equalizados
            frame_norm = cv2.merge((canal_azul_eq, canal_verde_eq, canal_vermelho_eq))

        if True:
            # Defina o kernel para dilatação e erosão
            kernel = np.ones((5, 5), np.uint8)

            # Aplique a dilatação
            imagem_dilatada = cv2.dilate(frame_norm, kernel, iterations=1)

            # Aplique a erosão
            frame_norm = cv2.erode(imagem_dilatada, kernel, iterations=1)


        self.preenchimento(frame_norm, posicao_amostra, [255,0,0])

        
        frame_norm = cv2.cvtColor(frame_norm, cv2.COLOR_BGR2GRAY)
        
        pos_curve = 80
        dd = frame_norm[pos_curve:pos_curve+2,:][0]
        
        pista = []
        for idx in range(0,len(dd)):
            if dd[idx]==255:
                pista.append(idx)
            #else:
            #    pista.append(0)
        
        #print(pista)
        cg_pixel = 0
        if len(pista)==0:
            cg = 0
            cg_pixel = 0
        else:
            cg_pixel = int(np.mean(pista))
            cg = 100-cg_pixel
            if cg>100:
                cg = 100
            if cg<-100:
                cg=-100
            
        #frame_norm[80:82,:] = 255
        
        frame_norm = cv2.cvtColor(frame_norm, cv2.COLOR_GRAY2BGR)
        cv2.circle(frame_norm, posicao_amostra, 2, (255,0,0), cv2.FILLED, cv2.LINE_AA)
        cv2.circle(frame_norm, (cg_pixel, int(pos_curve)), 2, (0,255,0), cv2.FILLED, cv2.LINE_AA)
        frame_norm = cv2.resize(frame_norm, (640,360))

        
        steer = cg
        
        #print(cg)
        
        
        return frame_norm, steer, 0
        
        
        #return frame_norm, steer, 0
        
        while(len(self.smooth)>1):
            self.smooth.pop(0)
        
        steer = np.mean(self.smooth)
        
        return frame_norm, int(steer), int(speed)

        

serialPort = "/dev/ttyUSB0"

display = DisplaySSD1306()

display.write(0,0, "Carregando\nModelo....")
full_drive = FullDrive()

display.write(0,0, "Iniciando\nCamera....")
camera = Camera()

display.write(0,0, "Iniciando\nChassis....")
chassis = CarComm(serialPort)

display.write(0,0, "Iniciando\nRecordData....")
record_data = RecordData()

display.write(0,0, "Pronto!")

last_mode = None
idx_learning = 0
fps = 0
start_time = datetime.datetime.now()
waiting_speed = True
steer_test = 0

#online_view = OnlineView()
# inicia arquivo de video...
file_name = "debug_{}".format(datetime.datetime.now())
record_data.startFiles(file_name)


while True:
    try:
        if chassis.mode != last_mode:
            print(last_mode, ">", chassis.mode)

            if chassis.mode == 1:
                print("Manual")
                display.write(0,0, "Manual")
                #time.sleep(2)

            elif chassis.mode == 2:
                print("Semi Auto")
                display.write(0,0, "Semi\nAuto")
                #time.sleep(2)

                

            elif chassis.mode == 3:
                print("Full Drive")
                display.write(0,0, "Full\nDrive")
                #time.sleep(3)
                

            elif chassis.mode == 4:
                print("Learning Mode")
                display.write(0,0, "Learning\nMode")
                #time.sleep(3)
                


                # inicia arquivo de video...
                file_name = "train_{}".format(datetime.datetime.now())

                # inicia arquivo csv...
                #record_data.startFiles(file_name)

                idx_learning = 0

            #print(last_mode,  chassis.mode)
            if last_mode==4 and chassis.mode!=4:
               
                
                record_data.closeFiles()
                
                print("Feito")

            
            last_mode = chassis.mode


        if (chassis.mode!=0) and (chassis.isOnline()==False) :
            print("timeout rasp")
            display.write(0,0, "Timeout\nRasp!!")
            time.sleep(1)


        if chassis.mode==4:
            #print(chassis.mode)
            # aprendendo...
            if chassis.speed>5:
                idx_learning+=1
                if waiting_speed:
                    display.write(0,0, "Registrando...")
                    waiting_speed=False

                # captura frame
                frame = camera.get_frame()
                
                if np.all(frame)!=None:
                    # registra dado csv.
                    data = [idx_learning, chassis.mode, chassis.speed, chassis.steer, chassis.sonar_r, chassis.sonar_c, chassis.sonar_l]
                    

                    
                    record_data.write(data, frame)
                else:
                    display.write(0,0, "Falha\nCam")
                    time.sleep(1)
                
                fps+=1
                if (datetime.datetime.now()-start_time).total_seconds()>=1:
                    print("FPS:{}".format(fps))
                    start_time = datetime.datetime.now()
                    fps = 0
                    record_data.checkBuffer()
            else:
                display.write(0,0, "Acelere!\nidx:{}".format(idx_learning))
                waiting_speed = True
                time.sleep(0.5)


        if chassis.mode in [2,3]:
            # captura frame
            #print("Capturando...")
            steer, speed = 0,0
            frame = camera.get_frame()

            if np.all(frame)!=None:
                #print("get_action...")
                frame_processed, steer, speed = full_drive.get_action(frame)

                chassis.set_steer(steer)
                chassis.set_speed(0)

                record_data.write([steer], frame_processed)

                #print(f"{steer}")
                
                
                if False:
                    time.sleep(0.03)
                    steer_test+=1
                    if steer_test>90:
                        steer_test=-90
                    print(steer_test)
                    chassis.set_steer(steer_test)
                    chassis.set_speed(0)
                    
            else:
                print("Frame=Null")
                display.write(0,0, "Falha\nCam")
                chassis.set_steer(0)
                chassis.set_speed(0)
                time.sleep(1)
            

            fps+=1
            if (datetime.datetime.now()-start_time).total_seconds()>=1:
                print("FPS:{}".format(fps))
                #display.write_assync(0,0, "dir:{}\nFPS:{}".format(steer, fps))
                record_data.checkBuffer()
                #display.write_assync(0,0, "AUTONOMO ({})\nD: {} S: {}".format(fps, steer, speed))
                start_time = datetime.datetime.now()
                fps = 0
    except Exception as er:
        display.write(0,0, "ERR GERAL:\n{}".format(er))
        print("Erro Geral")
        print(er)
        time.sleep(1)

    time.sleep(0.001)

 
