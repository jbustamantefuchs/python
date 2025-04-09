import paho.mqtt.client as mqtt
import rospy
from std_msgs.msg import String
import signal
import sys


rospy.init_node("mqtt_to_ros", anonymous=True)
pub = rospy.Publisher("nr_controller", String, queue_size=10)


datos = {
    "direction": None,
    "speed": None
}


ultimo_estado = None


def on_connect(client, userdata, flags, rc):
    print("Connected with result: " + str(rc))
    client.subscribe("direction")
    client.subscribe("speed")


def on_message(client, userdata, msg):
    global ultimo_estado

    if msg.topic in datos:
        nuevo_valor = msg.payload.decode() or "0"
        if datos[msg.topic] != nuevo_valor:
            datos[msg.topic] = nuevo_valor
            print(f"Updated message on the topic {msg.topic}: {datos[msg.topic]}")
        else:
            return 

    estado_actual = (
        datos["direction"] if datos["direction"] is not None else "0",
        datos["speed"] if datos["speed"] is not None else "0"
    )

    if estado_actual != ultimo_estado:
        mensaje_ros = f"({estado_actual[0]},{estado_actual[1]})"
        pub.publish(mensaje_ros)
        print(f"Pub in ROS: {mensaje_ros}")
        ultimo_estado = estado_actual


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect("localhost", 1883, 60)


def signal_handler(sig, frame):
    print("\nClosing MQTT and ROS...")
    client.disconnect()
    rospy.signal_shutdown("Closing by Ctrl+C")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


try:
    client.loop_forever()
except KeyboardInterrupt:
    signal_handler(None, None)
