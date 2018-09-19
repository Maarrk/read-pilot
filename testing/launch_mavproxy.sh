MAP_SERVICE=GoogleMap
mavproxy.py --master=tcp:localhost:5760 --out=udp:localhost:10000 --out=udp:localhost:1234 --streamrate 10 --map --console #--out=/dev/ttyUSB0 --baudrate=115200
