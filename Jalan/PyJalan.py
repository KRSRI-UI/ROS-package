import serial # ini untuk komunikasi serial
import time # ini untuk ngatur delay

ser = serial.Serial('/dev/ttyACM0', baudrate = 9600, timeout = 1)
time.sleep(3)

commandList = ['a RobotForward(1)',
               'b RobotReverse(1)',
               'c RobotRotateRight(1)', 
               'd RobotRotateLeft(1)', 
               'e RobotRotateRightSmooth(1)', 
               'f RobotRotateLeftSmooth(1)', 
               'g ...(...)', # Baru sampe sini
               'h ...(...)', 
               'i ...(...)',
               'j ...(...)',
               'k ...(...)', 
               'l ...(...)', 
               'm ...(...)', 
               'n ...(...)', 
               'o ...(...)', 
               'p ...(...)', 
               'q ...(...)', 
               'r ...(...)', 
               's ...(...)', 
               't ...(...)', 
               'u ...(...)', 
               'v ...(...)', 
               'w ...(...)',
               ""
            ]


def printMenu():
    print("**********************************************************"*2)
    print("MENU OPTIONS:")
    for i in range(0, len(commandList)-1, 2):
        part1 = chr(i+97)+"). "+commandList[i].ljust(45, " ")
        print(part1+chr(i+1+97)+"). "+commandList[i+1]+"\t\t ")
    print("[*] TYPE \"MENU\" to show the command menu[*]")
    print("[*] TYPE \"QUIT\" to quit[*]")
    print("**********************************************************"*2)
    print("")

def fungsi(opsi):
    kirim = ';' + str(opsi) + ';$'
    ser.write(kirim.encode('ascii'))
    terima = ser.readline().decode('ascii')
    return terima

ex = 1
printMenu()
while(ex == 1):
    try:
        v = input("Enter cmd letter:")
        if(v == "QUIT"):
            ex = 0
            print("QUITING")
            break
        if(v == "MENU"):
            printMenu()
            continue
        print('[*] COMMAND -> '+commandList[ord(v)-97] + '[*]')
        print("[*] RESPONSE [*]")
        numEnter = v
        v = v[0].lower()
        if(v == 'a'):
            print("\n\t"+fungsi(numEnter))

        elif(v == 'b'):
            print("\n\t"+fungsi(numEnter))

        elif(v == 'c'):
            print("\n\t"+fungsi(numEnter))

        elif(v == 'd'):
            print("\n\t"+fungsi(numEnter))

        elif(v == 'e'):
            print("\n\t"+fungsi(numEnter))

        elif(v == 'f'):
            print("\n\t"+fungsi(numEnter))

        elif(v == 'g'):
            print("\n\t"+str("Done"))

        elif(v == 'h'):
            print("\n\t"+str("Done"))

        elif(v == 'i'):
            print("\n\t"+str("Done"))

        elif(v == 'j'):
            print("\n\t"+str("Done"))

        elif(v == 'k'):
            print("\n\t"+str("Done"))

        elif(v == 'l'):
            print("\n\t"+str("Done"))

        elif(v == 'm'):
            print("\n\t"+str("Done"))

        elif(v == 'n'):
            print("\n\t"+str("Done"))

        elif(v == 'o'):
            print("\n\t"+str("Done"))

        elif(v == 'p'):
            print("\n\t"+str("Done"))

        elif(v == 'q'):
            print("\n\t"+str("Done"))

        elif(v == 'r'):
            print("\n\t"+str("Done"))

        elif(v == 's'):
            print("\n\t"+str("Done"))

        elif(v == 't'):
            print("\n\t"+str("Done"))

        elif(v == 'u'):
            print("\n\t"+str("Done"))

        elif(v == 'v'):
            print("\n\t"+str("Done"))

        elif(v == 'w'):
            print("\n\t"+str("Done"))

        print("")
    except KeyboardInterrupt:
        print("\nQUITING")
        quit()
    # except TypeError:
    #     print("Please enter only a single letter")
    except IndexError:
        print(f"Command {v} not found")
    except Exception as e:
        # General error -> just print it
        print(f"Error {e}")


