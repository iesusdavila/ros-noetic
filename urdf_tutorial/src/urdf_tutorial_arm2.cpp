#include <ros/ros.h>
#include <stdio.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <urdf_tutorial/changecontrolledjoints.h>

double deltaElbowPitch;
double deltaWristPitch;
double scale;
const double degree2rad = M_PI/180;
double elbowPitch = 0.0;
double wristPitch = 0.0;

int32_t c1 = 5;
int32_t c2 = 3;

// Declaración de mensajes
sensor_msgs::JointState joint_state;

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

bool changetJointController(urdf_tutorial::changecontrolledjoints::Request &req,
                            urdf_tutorial::changecontrolledjoints::Response &res)
{
    if (req.c1 > 0 && req.c1 <= 8 && req.c2 > 0 && req.c2 <= 8 && req.c1 != req.c2) {
        c1 = req.c1;
        c2 = req.c2;

        return true;
    } else {
        ROS_ERROR("Error in joint selection validation. Please select two different joint indices between 1 and 8.");
        return false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "urdf_tutorial");
    ros::NodeHandle n;

    // El nodo se suscribe al tema "teleop_values" para recibir comandos de teleoperación
    ros::Subscriber teleop_sub = n.subscribe("teleop_values", 1, teleopCallback);

    // El nodo publica los valores de los joints elbowPitch-wristPitch
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

    ros::ServiceServer service = n.advertiseService("set_joints_controller", changetJointController);

    ros::Rate loop_rate(30);

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
        joint_state.name[0] = "shoulder_pan_joint";
        joint_state.name[1] = "shoulder_pitch_joint";
        joint_state.name[2] = "elbow_roll_joint";
        joint_state.name[3] = "elbow_pitch_joint";
        joint_state.name[4] = "wrist_roll_joint";
        joint_state.name[5] = "wrist_pitch_joint";
        joint_state.name[6] = "gripper_roll_joint";
        joint_state.name[7] = "finger_joint1";
        joint_state.name[8] = "finger_joint2";

        // Update joint names and positions based on the requested joint indices
        for (int i = 0; i < 9; i++) {            
            joint_state.position[i] = (i == c1) ? elbowPitch : (i == c2) ? wristPitch : 0.0;
        }

        // Publica el estado de los joints
        joint_pub.publish(joint_state);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}