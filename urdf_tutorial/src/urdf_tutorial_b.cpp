#include <ros/ros.h>
#include <stdio.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <urdf_tutorial/changescale.h> // Include the service message header file

double deltaPan;
double deltaTilt;
double scale;
const double degree2rad = M_PI/180;
double pan = 0.0;
double tilt = 0.0;

const std::string MENSAJE = "Control de articulaciones:\n"
                            "--------------------------------------------------\n"
                            "Presiona 'arriba' para inclinar hacia arriba tilt_joint\n"
                            "Presiona 'abajo' para inclinar hacia abajo tilt_joint\n"
                            "Presiona 'derecha' para girar a la izquierda pan_joint\n"
                            "Presiona 'izquierda' para girar a la derecha pan_joint\n"
                            "Presiona 'q' para salir\n";

// Callback para procesar los comandos de teleoperación
void teleopCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // Ajusta los deltas de pan y tilt según los comandos recibidos
    deltaPan = msg->angular.z * scale;
    deltaTilt = msg->linear.x * scale;

    // Actualiza los valores de pan y tilt
    pan += deltaPan;
    tilt += deltaTilt;
}

// Callback para procesar el servicio de cambio de escala
bool changetScale(urdf_tutorial::changescale::Request &req, urdf_tutorial::changescale::Response &res)
{
    scale = req.scale; // Actualiza el valor de la escala
    res.success = true; // Devuelve un booleano indicando que la operación fue exitosa
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "urdf_tutorial");
    ros::NodeHandle n;

    // El nodo se suscribe al tema "teleop_values" para recibir comandos de teleoperación
    ros::Subscriber teleop_sub = n.subscribe("teleop_values", 1, teleopCallback);

    // El nodo publica los valores de los joints pan-tilt
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

    // Crea el servicio de cambio de escala
    ros::ServiceServer service = n.advertiseService("set_scale", changetScale);

    ros::Rate loop_rate(30);

    // Declaración de mensajes
    sensor_msgs::JointState joint_state;
    joint_state.name.resize(2);
    joint_state.position.resize(2);

    deltaPan = 0.0;
    deltaTilt = 0.0;
    scale = 0.10;

    while (ros::ok())
    {
        std::cout << MENSAJE << std::endl;
        std::cout << "Pan: " << pan << ", Tilt: " << tilt << std::endl;

        // Actualiza el estado de los joints
        joint_state.header.stamp = ros::Time::now();
        joint_state.name[0] = "pan_joint";
        joint_state.position[0] = pan;
        joint_state.name[1] = "tilt_joint";
        joint_state.position[1] = tilt;

        // Publica el estado de los joints
        joint_pub.publish(joint_state);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}