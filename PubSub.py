# Import standard python modules.
import sys
import time
import serial
 

# This example uses the MQTTClient instead of the REST client
from Adafruit_IO import MQTTClient

# holds the count for the feed
global run_count

#ADAFRUIT_IO_USERNAME = "----"
#ADAFRUIT_IO_KEY      = "---"

# Set to the ID of the feed to subscribe to for updates.
#feedContador = 'contador'
feedsev1 = 'Servomotor1'
feedsev2 = 'Servomotor2'
feedsev3 = 'Servomotor3'
feedsev4 = 'Servomotor4'


mensaje = ''

# Define callback functions which will be called when certain events happen.
def connected(client):
    """Connected function will be called when the client is connected to
    Adafruit IO.This is a good place to subscribe to feed changes.  The client
    parameter passed to this function is the Adafruit IO MQTT client so you
    can make calls against it easily.
    """
    # Subscribe to changes on a feed named Counter.
    #print('Subscribing to Feed {0} and {1}'.format(feedLed, feedContador))
    client.subscribe(feedsev1)
    client.subscribe(feedsev2)
    client.subscribe(feedsev3)
    client.subscribe(feedsev4)
    print('Waiting for feed data...')

def disconnected(client):
    """Disconnected function will be called when the client disconnects."""
    sys.exit(1)

def message(client, feed_id, payload):
    """Message function will be called when a subscribed feed has a new value.
    The feed_id parameter identifies the feed, and the payload parameter has
    the new value.
    """
    print('Feed {0} received new value: {1}'.format(feed_id, payload))
    
    if(feed_id == feedsev1):
        arduino.write(bytes('1', 'utf-8'))
        arduino.write(bytes((payload.rjust(3,'0')), 'utf-8'))
    if(feed_id == feedsev2):
        arduino.write(bytes('2', 'utf-8'))
        arduino.write(bytes((payload.rjust(3,'0')), 'utf-8'))
    if(feed_id == feedsev3):
        arduino.write(bytes('3', 'utf-8'))
        arduino.write(bytes((payload.rjust(3,'0')), 'utf-8'))
    if(feed_id == feedsev4):
        arduino.write(bytes('4', 'utf-8'))
        arduino.write(bytes((payload.rjust(3,'0')), 'utf-8'))
        


    


try:
    client = MQTTClient(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

    # Setup the callback functions defined above.
    client.on_connect = connected
    client.on_disconnect = disconnected
    client.on_message = message

    # Connect to the Adafruit IO server.
    client.connect()
    client.loop_background()
              
    arduino = serial.Serial(port='COM4', baudrate =9600, timeout = 0.1)
    
    while True:
        #mensaje = arduino.readline().decode('utf-8')
        #print(mensaje)
        time.sleep(0.1)
        
        
except KeyboardInterrupt:
    print("Salimos del programa")
    if arduino.is_open:
        arduino.close()
    sys.exit(1)