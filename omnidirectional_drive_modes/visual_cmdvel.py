import matplotlib.pyplot as plt
import numpy as np
import rospy
from geometry_msgs.msg import Twist

# Inicializar la figura y el eje de matplotlib con una ventana más pequeña (0 a 60)
fig, ax = plt.subplots()
ax.set_xlim(0, 60)
ax.set_ylim(0, 60)

# Definir el tamaño y la posición inicial del cuadrado (más grande)
square_size = 30  # Tamaño más grande del cuadrado
x, y = 30, 30  # Posición inicial en el centro de la pantalla
angle = 0  # Ángulo de rotación en grados

# Definir la dirección y el color del cuadrado
linear_x = 0.0
linear_y = 0.0
angular_z = 0.0
color = (1, 0, 0)  # Rojo en RGB normalizado

# Función para suscribirse a cmd_vel
def cmd_vel_callback(msg):
    global linear_x, linear_y, angular_z, color

    linear_x = msg.linear.x
    linear_y = msg.linear.y
    angular_z = msg.angular.z

    # Calcular color basado en el valor absoluto de los componentes
    max_value = 18.0
    value = max(abs(linear_x), abs(linear_y), abs(angular_z))

    if value == 0:
        color = (1, 0, 0)  # Rojo
    elif value <= max_value:
        # Escalar entre rojo y azul
        red = max(0, 1 - (value / max_value))
        blue = value / max_value
        green = (red + blue) / 2  # Mezclar rojo y azul para verde
        color = (red, green, blue)

# Suscribirse a /cmd_vel
rospy.init_node('square_movement', anonymous=True)
rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)

# Crear el cuadrado como un objeto de matplotlib
square = plt.Rectangle((x - square_size / 2, y - square_size / 2), square_size, square_size, color=color)
ax.add_patch(square)

# Función para mover y rotar el cuadrado
def update_square():
    global x, y, angle, linear_x, linear_y, angular_z, color

    # Si linear.x, linear.y, y angular.z son 0, restablecer a la posición y ángulo inicial
    if linear_x == 0 and linear_y == 0 and angular_z == 0:
        x, y = 30, 30  # Restablecer al centro de la ventana
        angle = 0  # Restablecer ángulo

    # Mover el cuadrado según las entradas de linear.x y linear.y (con un pequeño factor de escala)
    move_scale = 1  # Factor de escala para el movimiento (más pequeño para movimiento suave)
    if linear_x > 0:
        y += move_scale  # Mover hacia arriba
    elif linear_x < 0:
        y -= move_scale  # Mover hacia abajo

    if linear_y > 0:
        x += move_scale  # Mover hacia izquierda (invertir y)
    elif linear_y < 0:
        x -= move_scale  # Mover hacia derecha (invertir y)

    # Asegurarse de que el cuadrado no se salga de la pantalla
    x = max(square_size / 2, min(x, 60 - square_size / 2))
    y = max(square_size / 2, min(y, 60 - square_size / 2))

    # Actualizar el color basado en los valores actuales
    square.set_facecolor(color)

    # Actualizar la posición del cuadrado
    square.set_xy((x - square_size / 2, y - square_size / 2))

    # Si angular.z es 0, restablecer el ángulo de rotación a 0
    if angular_z == 0:
        angle = 0
    else:
        angle += angular_z * 2  # Controlar la velocidad de rotación

    # Aplicar la rotación en torno al centro del cuadrado
    t = plt.gca().transData
    rotate = plt.matplotlib.transforms.Affine2D().rotate_deg_around(x, y, angle)
    square.set_transform(rotate + t)

# Bucle de actualización
while not rospy.is_shutdown():
    update_square()

    # Limpiar y actualizar la figura
    plt.draw()
    plt.pause(0.05)  # Pausa para la actualización

# Cerrar la ventana de matplotlib al finalizar
plt.close()

