########## TO RUN SCRIPT ##########
#In terminal, type: python script.py

##############
## Script listens to serial port and writes contents into a file
##############
## requires pySerial to be installed 
import serial

#Change the serial_port value if not an Arduino Mega 
#Go to your arduino code and in the main menu, go to  
#Tools then port and choose the path of the nonbluetooth port
serial_port = '/dev/cu.usbmodem1421'; 

baud_rate = 9600; #In arduino, Serial.begin(baud_rate)

#We can write directly to the .csv file  
#Change the path to where you want your computer to write the output to
write_to_file_path = "/Users/taylorpiggy/Desktop/SCUSolarCar/Nanogrid/VoltageCurrent_SenseTest/output.csv";

output_file = open(write_to_file_path, "w+");
ser = serial.Serial(serial_port, baud_rate)
while True:
    line = ser.readline();
    line = line.decode("utf-") #ser.readline returns a binary, convert to string
    print(line);
    output_file.write(line);
