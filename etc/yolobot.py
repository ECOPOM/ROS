from __future__ import print_function
import roslibpy
import json

client = roslibpy.Ros(host='10.60.64.1', port=9090)
client.run()


data={}
def receive_message(msg):
    data['detections'] = msg['data']
    # print('Heard talking: ' + str(msg['data']))

listener = roslibpy.Topic(client, '/yolobot/detections', 'std_msgs/Int8')
listener.subscribe(receive_message)

try:
    while True:
        print(json.dumps(data, indent=2))
except KeyboardInterrupt:
    client.terminate()
