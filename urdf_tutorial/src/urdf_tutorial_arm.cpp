#include <ros/ros.h>
#include <stdio.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

double deltaElbowPitch;
double deltaWristPitch;
double scale;
const double degree2rad = M_PI/180;
double elbowPitch = 0.0;
double wristPitch = 0.0;

const std::string MENSAJE = "Control de articulaciones:\n"
                            "--------------------------------------------------\n"
                            "Presiona 'arriba' para inclinar hacia arriba wrist_pitch_joint\n"
                            "Presiona 'abajo' para inclinar hacia abajo wrist_pitch_joint\n"
                            "Presiona 'derecha' para girar a la izquierda elbow_pitch_joint\n"
                            "Presiona 'izquierda' para girar a la derecha elbow_pitch_joint\n"
                            "Presiona 'q' para salir\n";

// Callback para procesar los comandos de teleoperación
void teleopCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  // Ajusta los deltas de elbowPitch y wristPitch según los comandos recibidos
  deltaElbowPitch = msg->angular.z * scale;
  deltaWristPitch = msg->linear.x * scale;

  // Actualiza los valores de elbowPitch y wristPitch
  elbowPitch += deltaElbowPitch;
  wristPitch += deltaWristPitch;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "urdf_tutorial");
  ros::NodeHandle n;

  // El nodo se suscribe al tema "teleop_values" para recibir comandos de teleoperación
  ros::Subscriber teleop_sub = n.subscribe("teleop_values", 1, teleopCallback);

  // El nodo publica los valores de los joints elbowPitch-wristPitch
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

  ros::Rate loop_rate(30);

  // Declaración de mensajes
  sensor_msgs::JointState joint_state;
  joint_state.name.resize(9);
  joint_state.position.resize(9);

  deltaElbowPitch = 0.0;
  deltaWristPitch = 0.0;
  scale = 0.10;

  while (ros::ok())
  {
    std::cout << MENSAJE << std::endl;
    std::cout << "elbowPitch: " << elbowPitch << ", wristPitch: " << wristPitch << std::endl;

    // Actualiza el estado de los joints
    joint_state.header.stamp = ros::Time::now();
    joint_state.position[0] = 0;
    joint_state.name[0] = "shoulder_pan_joint";
    joint_state.position[1] = 0;
    joint_state.name[1] = "shoulder_pitch_joint";
    joint_state.position[2] = 0;
    joint_state.name[2] = "elbow_roll_joint";
    joint_state.position[3] = elbowPitch;
    joint_state.name[3] = "elbow_pitch_joint";
    joint_state.position[4] = 0;
    joint_state.name[4] = "wrist_roll_joint";
    joint_state.position[5] = wristPitch;
    joint_state.name[5] = "wrist_pitch_joint";
    joint_state.position[6] = 0;
    joint_state.name[6] = "gripper_roll_joint";
    joint_state.position[7] = 0;
    joint_state.name[7] = "finger_joint1";
    joint_state.position[8] = 0;
    joint_state.name[8] = "finger_joint2";    

    // Publica el estado de los joints
    joint_pub.publish(joint_state);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
