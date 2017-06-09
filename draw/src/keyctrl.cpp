//
// Created by sun on 17-6-9.
//

#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <moveit/move_group_interface/move_group.h>

#define KEYCODE_R 0x61
#define KEYCODE_L 0x64
#define KEYCODE_U 0x77
#define KEYCODE_D 0x73
#define KEYCODE_Q 0x71

int kfd = 0;
struct termios cooked, raw;

class KeyctrlRobot {
public:
    KeyctrlRobot();

    ~KeyctrlRobot();

    void keyLoop();

private:
//    ros::NodeHandle nh_;
//    ros::Publisher maker_pub_;
//    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;
};

KeyctrlRobot::KeyctrlRobot() {

}

KeyctrlRobot::~KeyctrlRobot() {

}
int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}
void KeyctrlRobot::keyLoop() {
    char c;
    bool dirty = false;

    //moveit config
    geometry_msgs::Pose pose;
    moveit::planning_interface::MoveGroup group("manipulator");
    move_group_interface::MoveGroup::Plan plan;

    for (;;) {
        // get the next event from the keyboard
        c = (char) getkey();

        pose = group.getCurrentPose().pose;
//        ROS_INFO("X:%lf , Y:%lf, Z:%lf",pose.position.x,pose.position.y,pose.position.z);


        switch (c) {
            case KEYCODE_L:
                ROS_INFO("Y-AXIS -");
                pose.position.y -=0.2;
                dirty = true;
                break;
            case KEYCODE_R:
                ROS_INFO("Y-AXIS +");
                pose.position.y +=0.2;
                dirty = true;
                break;

            case KEYCODE_U:
                ROS_INFO("X-AXIS -");
                pose.position.x -=0.2;
                dirty = true;
                break;
            case KEYCODE_D:
                ROS_INFO("X-AXIS +");
                pose.position.x +=0.2;
                dirty = true;
                break;
        }



        if (dirty == true) {
            ROS_INFO("Moving the Robot");
            group.setPoseTarget(pose);
            group.setPlanningTime(10.0f);
            if (!group.plan(plan))
            {
                ROS_FATAL("Unable to create motion plan.  Aborting.");
                exit(-1);
            }
            moveit_msgs::MoveItErrorCodes result = group.asyncExecute(plan);
//            moveit_msgs::MoveItErrorCodes result = group.move();
            if(result.val != result.SUCCESS){
                ROS_ERROR_STREAM("Move failed");
            }
            dirty = false;
        }
    }

    return;
}
void quit(int sig)
{
   tcsetattr(kfd, TCSANOW, &cooked);
   ros::shutdown();
   exit(0);
}
int main(int argc, char** argv){
    ros::init(argc , argv, "key_ctrl");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    KeyctrlRobot keyctrl;
    signal(SIGINT, quit);
    keyctrl.keyLoop();
    return 0;
}
