void takeoff(){
    system("bash -i -c 'rostopic pub /ardrone/takeoff std_msgs/Empty -1'");
}
void land(){
    system("bash -i -c 'rostopic pub /ardrone/land std_msgs/Empty -1'");
}
void z(){
    system("bash -i -c 'rostopic pub /cmd_vel geometry_msgs/Twist -1 \"{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: -0.7}}\"'");
}
void nomove(){
    system("bash -i -c 'rostopic pub /cmd_vel geometry_msgs/Twist -1 \"{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}\"'");
}
void reset(){
    system("bash -i -c 'rostopic pub /ardrone/reset std_msgs/Empty -1'");
}

int main()
{
    takeoff();
    z();
    nomove();
    land();
    reset();

    return 0;
}
