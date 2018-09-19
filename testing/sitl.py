from time import sleep
from dronekit import connect
import dronekit_sitl

print "Start simulator (SITL)"
# Pobiednik
sitl = dronekit_sitl.start_default(50.090813, 20.199227)
# pobiednik geofence nad betonem - home po wschodniej stronie 
# sitl = dronekit_sitl.start_default(50.084591, 20.204530)
# pobiednik testowy
# sitl = dronekit_sitl.start_default(50.084692, 20.198571)
connection_string = sitl.connection_string()

print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)

vehicle.parameters['BATT_CAPACITY']=100000.0

sleep(5)

vehicle.close()

sleep(6000)

# Shut down simulator
sitl.stop()
print("Completed")
