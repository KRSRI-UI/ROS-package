import serial # ini untuk komunikasi serial
import time # ini untuk ngatur delay
from huskylib import HuskyLensLibrary

ser = serial.Serial('COM7', baudrate = 9600, timeout = 1)
time.sleep(3)

def fungsi(opsi):
    kirim = ';' + str(opsi) + ';$'
    ser.write(kirim.encode('ascii'))
    terima = ser.readline().decode('ascii')
    return terima





# ============================ START SETUP ============================
hl = HuskyLensLibrary("I2C","", address=0x32)
step = 0
# ============================= END SETUP =============================





# ============================ START LOOP 0 ============================
loop0 = True
while(loop0):

    # Switch the algorithm to Object Classification
    hl.algorthim("ALGORITHM_OBJECT_CLASSIFICATION")

    # Load Learned Object Classification Data From Memory
    hl.loadModelFromSDCard(1)                                       # ..Model..(1) = Obj. Class  |  ..Model..(2) = Obj. Track. Korban  |  ..Model..(3) = Obj. Track. Lilin.

    # ========================== START LOOP 01 =========================
    loop01 = True
    while(loop01):

        #Apakah Ada Learned Object == Lilin
        if (hl.blocks().ID == 2 and step % 2 == 0):                                       # ID(0) = Objek Belum Terpelajari  |  ID(1) = Korban  |  ID(2) = Lilin.

            # Switch the algorithm to Object Tracking
            hl.algorthim("ALGORITHM_OBJECT_TRACKING")
            
            # Load Learned Object Tracking Lilin Data From Memory
            hl.loadModelFromSDCard(3)

            # ===================== START LOOP 011 =====================
            loop011 = True
            while (loop011):
            
                # Apakah 110 <= blocks().width <= 130?                      
                if (110 <= hl.blocks().width and hl.blocks().width <= 130):

                    # Robot Melakukan Pemadaman
                    numEnter = 'h'                          # h = pemadaman lilin
                    print(fungsi(numEnter))
                    time.sleep(2)

                    # Out From LOOP 011 & LOOP 01
                    loop011 = False
                    loop01 = False
                    
                    # Next step
                    step += 1

                # Else, Apakah blocks().width < 110?
                elif (hl.blocks().width < 110):

                    # Robot Bergerak Ke Depan 1 Langkah
                    numEnter = 'a'
                    print(fungsi(numEnter))
                    time.sleep(2)

                # Else
                else:

                    #Robot Bergerak Ke Belakang 1 Langkah
                    numEnter = 'b'
                    print(fungsi(numEnter))
                    time.sleep(2)
            # ====================== END LOOP 011 ======================
        # Apakah Ada Learned Object == Korban
        elif (hl.blocks().ID == 1):
            
            # Switch the algorithm to Object Tracking
            hl.algorthim("ALGORITHM_OBJECT_TRACKING")
            
            # Load Learned Object Tracking Korban Data From Memory
            hl.loadModelFromSDCard(2)
            
            # ===================== START LOOP 012 =====================
            loop012 = True
            while(loop012):
            
                # Apakah 140 <= Blocks().x <= 160?
                if (140 <= hl.blocks().x and hl.blocks().x <= 160):
                
                    # Apakah 110 <= Blocks().width <= 130?
                    if (110 <= hl.blocks().width and hl.blocks().width <= 130):
                    
                        # Robot Bergerak Turun, Menjalankan Fungsi Mengambil Korban, Kembali Naik, Evakuasi
                        numEnter = 'i'                      # i = penyelamatan korban
                        print(fungsi(numEnter))
                        time.sleep(2)

                        # Out From LOOP 012 & LOOP 01
                        loop012 = False
                        loop01 = False

                        # Next step
                        step += 1

                    # Else, Apakah blocks().width < 110?            # < 110 = jauh
                    elif (hl.blocks().width < 110):
                        
                        # Robot Bergerak Ke Depan 1 Langkah
                        numEnter = 'a'
                        print(fungsi(numEnter))
                        time.sleep(2)

                    # Else
                    else:

                        # Robot Bergerak Ke Belakang 1 Langkah
                        numEnter = 'b'
                        print(fungsi(numEnter))
                        time.sleep(2)
            
                # Else, Apakah X.Objek < 140?
                elif (hl.blocks().x < 140):

                    # Robot Bergerak Ke Kanan 1 Langkah
                    numEnter = 'e'
                    print(fungsi(numEnter))
                    time.sleep(2)

                # Else
                else:

                    # Robot Bergerak Ke Kiri 1 Langkah
                    numEnter = 'f'
                    print(fungsi(numEnter))
                    time.sleep(2)
            # ====================== END LOOP 012 ======================
    # =========================== END LOOP 01 ==========================
# ============================= END LOOP 0 =============================