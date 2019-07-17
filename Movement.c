#include <kipr/botball.h>
#include <Movement.h>
////////////////////////////////////////////////////////////////////////////
int fast = 1;
int slow = 2;
int white = 1;
int black = 2;
double bias = 0.0;
//int arm_max = 1700;
//int arm_max = 1700;
int arm_valve = arm_max - 225;												//robot values
int arm_mats = arm_max - 105;
int arm_min = 1700 - 1600;
int hand_max = 1700;
int hand_tight = 1700 - 980;
int hand_min = 1700 - 920;
int hand_mats = 1700 - 900;
int hand_prism_close;
int hand_start = 1700;
int wireStarting = wireVertical + 520;
int wireDown = wireVertical - 1450;
int wireMagnet = wireVertical - 600;
int wirePvcNoRub = wireVertical - 610;
////////////////////////////////////////////////////////////////////////////

int total;
int i;
int buffer(int sensor)
{
    total = 0;
    i = 0;														//buffer for all sensors
    while(i<5){
        i = i+1;
        total = total + analog(sensor);
        msleep(1);
    }
    return(total/5);
}
////////////////////////////////////////////////////////////////////////////
void move(int l_speed,int r_speed){
    mav(R_motor,r_speed);						//simple move function using mav
    mav(L_motor,l_speed);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void square_up(int ending,int speed){
    if(speed > 0 && speed < 600){
        black_speed = 1.2*speed;
    }																		//square up function with both endings as far as white or black
    else{black_speed = 1.2*speed;}
    if(ending == 1 || ending == 2){
        while(1){
            if(analog(left_IR)<analog_white && analog(right_IR)<analog_white){
                move(speed,speed);
            }
            if(analog(right_IR)>analog_white){
                move(speed,stop);
            }
            if(analog(left_IR)>analog_white){
                move(stop,speed);
            }
            if(analog(left_IR)>analog_white && analog(right_IR)>analog_white) {
                move(stop,stop);
                break;
            }
        }
    }
    if(ending == 3){
        ao();
    }
    switch(ending){
        case 3:
            {
                while(1){
                    if(digital(digital_right)==0 && digital(digital_left)==0){
                        move(speed,speed);
                    }
                    if(digital(digital_right)==1){											
                        move(speed,stop);
                    }
                    if(digital(digital_left)==1){
                        move(stop,speed);
                    }
                    if(digital(digital_right)==1 && digital(digital_left)==1){
                        move(stop,stop);
                        break;
                    }
                }
            }
        case 1:
            {
                while(1){
                    if(analog(left_IR)>analog_white && analog(right_IR)>analog_white){
                        move(black_speed,black_speed);
                    }
                    if(analog(left_IR)<analog_white){
                        move(stop,black_speed);
                    }
                    if(analog(right_IR)<analog_white){
                        move(black_speed,stop);
                    }
                    if(analog(left_IR)<analog_white && analog(right_IR)<analog_white){
                        move(stop,stop);
                        break;
                    }
                }
            }
        case 2:
            {
                while(1){
                    if(analog(left_IR)>analog_white && analog(right_IR)>analog_white){
                        move(-1*black_speed,-1*black_speed);
                    }
                    if(analog(left_IR)<analog_white){
                        move(stop,-1*black_speed);
                    }
                    if(analog(right_IR)<analog_white){
                        move(-1*black_speed,stop);
                    }
                    if(analog(left_IR)<analog_white && analog(right_IR)<analog_white){
                        move(stop,stop);
                        break;
                    }
                }
            }
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double calibrate_gyro(){
    int i = 0;
    double avg = 0;
    while( i < 50){
        avg += gyro_z();
        msleep(1);
        i++;
        printf("Gyro Z: %d\n",gyro_z());						//gyro calibration for PID gyro drive and turning
    }
    bias = avg / 50.0;
    printf("New Bias: %f\n", bias);
    return bias;
}
////////////////////////////////////////////////////////////////////////////
void slow_arm (int x,int speed)//this funtion slows the servo movement
{ 

    enable_servo (arm);
    int desired_position;
    int current_position = get_servo_position(arm);
    if (0) {desired_position = arm_max;}
    else if(0) {desired_position = arm_min;}
    else {desired_position = x;}

    switch(speed){
        case 1:
            while(current_position <= desired_position-2 || current_position >= desired_position+2)
            {
                if(current_position < desired_position)
                { current_position=current_position +2;
                 set_servo_position(arm, current_position);
                 msleep(1);
                }
                if(current_position > desired_position)
                { current_position=current_position -2;
                 set_servo_position(arm, current_position);

                 msleep(1);
                }
            }
            break;
        case 2:
            while(current_position != desired_position)
            {
                if(current_position < desired_position)
                { current_position=current_position +1;
                 set_servo_position(arm, current_position);
                 msleep(2);
                }
                if(current_position > desired_position)
                { current_position=current_position -1;
                 set_servo_position(arm, current_position);

                 msleep(2);
                }
            }
            break;
    }

    set_servo_position (arm, x);
    msleep(15);
    disable_servo (arm);

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void slow_hand ( int z, int speed )//this funtion slows the hand servo
{ 

    enable_servo (hand);
    int desired_position;
    int variable;
    int current_position = get_servo_position(hand);
    if (0) {desired_position = hand_max;}
    else if(0) {desired_position = hand_min;}
    else {desired_position = z;}
	if (desired_position > current_position){
        variable = 3;
    }
	else{variable=2;}
    switch(speed){
        case 1:
            while(current_position <= desired_position-variable || current_position >= desired_position+variable)
            {
                if(current_position < desired_position)
                { current_position=current_position +variable;
                 set_servo_position(hand, current_position);
                 msleep(1);
                }
                if(current_position > desired_position)
                { current_position=current_position -variable;
                 set_servo_position(hand, current_position);
                 msleep(1);
                }
            }
            break;
        case 2:

            while(current_position != desired_position)
            {
                if(current_position < desired_position)
                { current_position=current_position +1;
                 set_servo_position(hand, current_position);
                 msleep(2);
                }
                if(current_position > desired_position)
                { current_position=current_position -1;
                 set_servo_position(hand, current_position);

                 msleep(2);
                }
            }
            break;
    }
    set_servo_position (hand, z);
    disable_servo (hand);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void a_cel(){
    float L_speed;
    float curr_time = seconds(); 
    float init_time = seconds();
    while( (curr_time - init_time) < .2 ){					//acceleration function using y = mx + b
        curr_time = seconds();   
        L_speed = 6.75 * ((curr_time - init_time) * 1000);
        move(L_speed,L_speed); 
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void de_cel(){


    cmpc(L_motor);
    cmpc(R_motor);
    while(gmpc(L_motor) < 500){

        int R_speed = 2.4 * (500 - gmpc(R_motor) + 150);        //deceleration that occours over 500 ticks can be adjusted
        int L_speed = 2.4 * (500 - gmpc(L_motor) + 150);
        move(L_speed,R_speed);

    }
    move(0,0);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int speed;
int distance;
double theta = 0;
void Drive(int desired){					//this is PID gyro drive adjusted with a_cel and d_cel moves for ticks(aka desired)

    cmpc(L_motor);
    cmpc(R_motor);
    a_cel();
    //desired = gmpc(L_motor) - desired;
    //desired = desired -500;
    int speed =1350;
    if(desired < speed){
        speed = desired;   
    }
    while(gmpc(L_motor) < (desired-500)){
        if (speed < 0){
            mav(L_motor, (speed - (speed * (theta/98000))));            
            mav(R_motor, (speed + (speed * theta/98000)));
            msleep(10);

            theta += (gyro_z() - bias) * 10;
        }
        else{
            mav(R_motor, (speed - (speed * (theta/98000))));            
            mav(L_motor, (speed + (speed * theta/98000)));
            msleep(10);

            theta += (gyro_z() - bias) * 10;
        }
        //desired = desired - gmpc(L_motor);
    }
    de_cel();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Drive_ET(int distance){

    cmpc(L_motor);
    cmpc(R_motor);
    a_cel();									//PID gyro drive with a_cel and d_cel but drives for ET distance
    double speed =750;
    while(1){
        if(buffer(5)>distance+50 && buffer(5)<distance-50){
            speed = speed/1.035;
        }
        if(buffer(5)>distance+25 || buffer(5)<distance-25){break;}
        else if(buffer(5)<distance){
            mav(R_motor, (speed - (speed * (theta/98000))));            
            mav(L_motor, (speed + (speed * theta/98000)));
            msleep(10);
        }
        else if(buffer(5)>distance){
            mav(R_motor, -1*(speed - (speed * (theta/98000))));            
            mav(L_motor, -1*(speed + (speed * theta/98000)));
            msleep(10);
        }
        theta += (gyro_z() - bias) * 10;
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*void Servo(){

    int x; //controls servo 1
    int y; //servo 2
    int z; //servo 3
    while(1){
        double init_time = seconds();
        double elapsed_time = seconds()-init_time;
        if(get_servo_position(1)<x){x=x+2;}							was going to move servo to a position over time
        else if(get_servo_position(1)>x){x=x-2;}
        if(get_servo_position(2)<y && time>30){y=y+2;}
        else if(get_servo_position(2)>y && time<30){y=y-2;}
        if(get_servo_position(3)<z && time>35){z=z+2;}
        else if(get_servo_position(3)>z && time>35){z=z-2;}
    }
}*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void start_pos()//setup for starting box
{
    slow_arm(fast,arm_max);
    set_servo_position(1,hand_min);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void turn_with_gyro(int speed, int deg){			// turns using gyro
    //calibrate_gyro();
    double theta ;
    int targetTheta; 
    switch(deg){
        case 45:
            targetTheta = target_theta_45;
            move(speed,speed*-1);
            break;
        case 90:
            targetTheta = target_theta_90;
            move(speed,speed*-1);
            break;
        case 180:
            targetTheta = target_theta_180;
            move(speed,speed*-1);
            break;
        case 360:
            targetTheta = target_theta_360;
            move(speed,speed*-1);
            break;
        case -45:
            targetTheta = target_theta_m45;
            move(speed*-1,speed);
            break;
        case -90:
            targetTheta = target_theta_m90;
            move(speed*-1,speed);						
            break;
        case -180:
            targetTheta = target_theta_m180;
            move(speed*-1,speed);
            break;
        case -360:
            targetTheta = target_theta_m360;
            move(speed*-1,speed);
            break;
        default:
            targetTheta = 0;
            break;
    }  
    while(theta < targetTheta){
        msleep(10);
        theta += abs(gyro_z() - bias) * 10;
        printf("Turn Gyro Z: %d\n",gyro_z());
    }
    mav(R_motor, 0);
    mav(R_motor, 0);
    ao();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Drive_SquareUp(int end,int desired){
    cmpc(L_motor);
    cmpc(R_motor);
    a_cel();
    //desired = gmpc(L_motor) - desired;
    //desired = desired -500;
    int speed =1350;									//drives for ticks then transfers to square up
    if(desired < speed){
        speed = desired;   
    }
    while(gmpc(L_motor) < (desired-500)){

        mav(R_motor, (speed - (speed * (theta/100000))));            
        mav(L_motor, (speed + (speed * theta/100000)));
        msleep(10);

        theta += (gyro_z() - bias) * 10;
        //desired = desired - gmpc(L_motor);
    }
    square_up(end, 500);

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PID_gyro_drive(int speed, double time){
    //calibrate_gyro(); commenting out for testing				PID drive for time
    double startTime = seconds();
    double theta = 0;
    while((seconds() - startTime) < time){
        if(speed > 0){
            mav(R_motor, (speed - (speed * (theta/96000))));            
            mav(L_motor, (speed + (speed * theta/96000)));
        }


        else{
            mav(L_motor, (speed - (speed * theta/97000)));            
            mav(R_motor, (speed + (speed * (theta/97000))));
        }
        msleep(10);
        theta += (gyro_z() - bias) * 10;
    }
    move(0,0);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*void PVC_wires(){
    
    int current_position = get_servo_position(2);
  
    while(current_position <= wirePvcNoRub-2 || current_position >= wirePvcNoRub+2) 	was a thread for e_lines
    {
        if(current_position < wirePvcNoRub)
        { current_position=current_position +2;
         set_servo_position(arm, current_position);
         msleep(1);
        }
        if(current_position > wirePvcNoRub)
        { current_position=current_position -2;
         set_servo_position(2, current_position);

         msleep(1);
        }   
    }
}*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Drive_LineFollow(int desired){//basic linefollow needs tuning
	
   cmpc(L_motor);
    cmpc(R_motor);
    a_cel();
    //desired = gmpc(L_motor) - desired;
    //desired = desired -500;
    int speed =1350;
    if(desired < speed){
        speed = desired;   
    }
    while(gmpc(L_motor) < (desired)){
        mav(R_motor,.2*(analog(2)));
        mav(L_motor,.2*(3400-analog(2)));
        
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void bulldoze_thread(){
     enable_servo (arm);
   
    int current_position = get_servo_position(arm);
            while(current_position <= arm_max-2 || current_position >= arm_max+2)   //thread for arm down
            {
                if(current_position < arm_max)
                { current_position=current_position +2;
                 set_servo_position(arm, current_position);
                 msleep(1);
                }
                if(current_position > arm_max)
                { current_position=current_position -2;
                 set_servo_position(arm, current_position);

                 msleep(1);
                }
            }
    set_servo_position (arm, arm_max);
    msleep(15);
    disable_servo (arm);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mat_water1(){			//thread 
     enable_servo (arm);
   
    int current_position = get_servo_position(arm);
            while(current_position <= 1490-2 || current_position >= 1490+2)
            {
                if(current_position < 1490)
                { current_position=current_position +2;
                 set_servo_position(arm, current_position);
                 msleep(1);
                }
                if(current_position > 1490)
                { current_position=current_position -2;
                 set_servo_position(arm, current_position);

                 msleep(1);
                }
            }
    set_servo_position (arm, 1490);
    msleep(15);
    disable_servo (arm);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mat_water_turn(){			//thread
     enable_servo (arm);
   msleep(50);
    int current_position = get_servo_position(arm);
            while(current_position <= 1040-2 || current_position >= 1040+2)
            {
                if(current_position < 1040)
                { current_position=current_position +3;
                 set_servo_position(arm, current_position);
                 msleep(1);
                }
                if(current_position > 1040)
                { current_position=current_position -3;
                 set_servo_position(arm, current_position);

                 msleep(1);
                }
            }
    set_servo_position (arm, 1040);
    msleep(15);
    disable_servo (arm);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Drive_ET_safe(int distance,int speed) 			//doesnt actually use ET uses IR to line up on line then linefollowa
{
   /*int init = analog(ET)-500;
    printf("%d",init);
    while( analog(5) < init + 100) //turn right until gain
    {
		move(200,-200);
        if (analog(ET) > init + 100){
         break;   
        }
    }
    while( analog(5) < init + 100) //turn right until gain
    {
		move(50,-50);
        if (analog(ET) > init + 100){
         break;   
        }
    }
    int blocks = analog(ET);
    printf("blocks reading is:%d",blocks);
    cmpc(L_motor);
    while(analog(ET) > ((blocks + init)/2)){
        printf("blocks reading is:%d",blocks);
        move(200,-200);
    }
    int pos = gmpc(L_motor)/2;
    cmpc(L_motor);
    while(gmpc(L_motor)>pos) //turn left until 1/2 gmpc
    {
        move(-50,50);
        if(gmpc(L_motor)==pos || gmpc(L_motor)<pos)
        {
            move(0,0);
            msleep(100);
            break;
        }
    }
    */
    if (analog(2)>1700)//if black rotate left
    {
        while(analog(2)>1402)
        {move(-100,100);
        }
        move(0,0);
        msleep(15);
    }
    if (analog(2)<1700)//if white rotate right
    {
     while(analog(2)<600)
        {move(100,-100);
        }
        move(0,0);
        msleep(15);
    }
  cmpc(L_motor);
    cmpc(R_motor);
    a_cel();
    
    while(1){
        if(analog(right_IR)<2500)//speed up or slow down wheels based on side IR and front IR
        {move(speed,speed*1.25);}
        else{move(speed,speed*.75);}
        if(analog(front_IR)<2500)
           {move(speed,speed*.75);}
           else{move(speed,speed*1.25);}
           if(gmpc(R_motor)>distance){break;}
                 
       /* if(buffer(5)>distance+50 && buffer(5)<distance-50){
            speed = speed/1.035;
        }
        if(buffer(5)<distance+25 && buffer(5)>distance-25){
            printf("buffer(5) is a failure and reads %d",buffer(5));
            break;}
        else if(distance > buffer(5)){
            mav(R_motor, (speed - (speed * (theta/100000))));            
            mav(L_motor, (speed + (speed * theta/100000)));
            msleep(10);
        }
        else if(distance < buffer(5)){
            mav(R_motor, -1*(speed - (speed * (theta/100000))));            
            mav(L_motor, -1*(speed + (speed * theta/100000)));
            msleep(10);
        }
        theta += (gyro_z() - bias) * 10;*/
    }
           move(0,0);
}
int collect_water()
{return 1;}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////