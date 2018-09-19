MAP_SERVICE=GoogleMap
mavproxy.py --master=/dev/ttyS0 --baudrate=115200 --out=tcpin:localhost:5760 --out=tcpin:localhost:14650 --streamrate 5 --map --console
