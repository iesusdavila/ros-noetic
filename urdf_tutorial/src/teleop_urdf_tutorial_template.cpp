#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <ncurses.h> // Incluye la biblioteca ncurses

double pan = 0.0;
double tilt = 0.0;
const double degree2rad = M_PI/180;
const double scale = 0.5; // Ajusta el valor de escala según tu preferencia

void updateJointState(sensor_msgs::JointState &joint_state, double pan, double tilt) {
    joint_state.header.stamp = ros::Time::now();
    joint_state.name[0] = "pan_joint";
    joint_state.position[0] = pan;
    joint_state.name[1] = "tilt_joint";
    joint_state.position[1] = tilt;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "urdf_tutorial");
    ros::NodeHandle n;

    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

    ros::Rate loop_rate(30);
    sensor_msgs::JointState joint_state;
    joint_state.name.resize(2);
    joint_state.position.resize(2);

    initscr(); // Inicializa la biblioteca ncurses
    cbreak();  // Habilita la entrada de teclado en modo sin búfer
    noecho();  // No muestra las teclas presionadas en la pantalla
    timeout(0); // No bloquea la entrada, 0 indica que no se bloqueará

    int ch;
    
    while (ros::ok()) {
        clear(); // Limpia la pantalla
        mvprintw(0, 0, "Control de articulaciones:");
        mvprintw(1, 0, "--------------------------------------------------");
        mvprintw(2, 0, "Presiona 'w' para inclinar hacia arriba pan_joint");
        mvprintw(3, 0, "Presiona 's' para inclinar hacia abajo pan_joint");
        mvprintw(4, 0, "Presiona 'a' para girar a la izquierda tilt_joint");
        mvprintw(5, 0, "Presiona 'd' para girar a la derecha tilt_joint");
        mvprintw(6, 0, "Presiona 'q' para salir");
        mvprintw(7, 0, "Pan: %f, Tilt: %f   ", pan, tilt);
        // Captura la tecla presionada
        ch = getch();
        if (ch != ERR) {
            switch (ch) {
                case 'w':
                    tilt += degree2rad * scale;
                    break;
                case 's':
                    tilt -= degree2rad * scale;
                    break;
                case 'a':
                    pan += degree2rad * scale;
                    break;
                case 'd':
                    pan -= degree2rad * scale;
                    break;
                default:
                    break;
            }

            updateJointState(joint_state, pan, tilt);
            joint_pub.publish(joint_state);
        }

        loop_rate.sleep();
    }

    endwin(); // Termina la sesión de ncurses

    return 0;
}
