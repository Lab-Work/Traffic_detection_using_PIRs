import os


#iparray = [116,107,108,109,110,113,114,115,105]
#iparray = [107,108,109,110,113,114,115,105]
#iparray = [103,107,109,105,110,113,114,115,116]
#iparray = [116,103,101,102,104,113,107,114,109]
#iparray = [102,104,113,107,114,109]
#iparray = [133,134,135,136,137,138]
iparray = [130,131,132]

for i in iparray:
    print "192.168.0.%s"%str(i)
    #os.system("ssh pi@192.168.0.%s \"cd Desktop/Sensor_Network/valpi/main; nohup sudo python validate.py &\" &"%str(i))
    os.system("ssh pi@192.168.0.%s \"cd Desktop/Sensor_Network; git pull https://modrie:poison123@github.com/Lab-Work/Sensor_Network\" &"%str(i))
    #os.system("ssh pi@192.168.0.%s \"rm -rf ~/Desktop/Sensor_Network/valpi/main/data/*\" &"%str(i))
    #os.system("ssh pi@192.168.0.%s \"sudo shutdown -r now\" &"%str(i))
    #os.system("ssh pi@192.168.0.%s \"cd Desktop/Sensor_Network; git pull https://modrie:poison123@github.com/Lab-Work/Sensor_Network; sudo apt-get update; sudo pip install xbee; sudo pip install pySerial\" &"%str(i))

    #os.system("ssh pi@192.168.0.%s hostname &"%str(i))
    #os.system("ssh pi@192.168.0.%s \"sudo pip install xbee, sudo pip install pySerial\""%str(i))
    #os.system("ssh pi@192.168.0.%s \"sudo raspi-config\""%str(i))
    #os.system("ssh pi@192.168.0.%s"%str(i))
    #os.system("ssh pi@192.168.0.%s \"sudo sed -i \\\"/enable_uart/ c\\enable_uart=1\\\" /boot/config.txt; sudo shutdown -r now\""%str(i))
    #os.system("ssh pi@192.168.0.%s \"sudo apt-get --assume-yes install python-dev\" &"%str(i))


    #os.system("ssh-copy-id pi@192.168.0.%s"%str(i))