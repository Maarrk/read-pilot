SET MAP_SERVICE=GoogleMap
"C:\Program Files (x86)\MAVProxy\mavproxy.exe" --master=tcp:localhost:5763 --out=udp:localhost:10000 --out=udp:localhost:3002 --streamrate 10 --map --console
