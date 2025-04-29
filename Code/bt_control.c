
#include "simpletools.h"                      // Include simple tools
#include "fdserial.h" //for bluetooth communication
#include "servo.h"
#include<propeller.h>

// global variables: left speed, right speed, stop?, gripper action, lift angle, grip angle, lift pin,  grip pin
volatile int ls,rs,stop=0,grip=0,a1=1800,a2=900,p1=14,p2=15;
int stack1[45+200], stack2[45+200],stack3[45+200],stack4[45+200];

void right(); //function for controlling right motor
void left(); //function for controlling right motor
void bt(); //function for handling bluetooth communication
void gripper(); //function for actuating gripper mechanism

int main()                                    // Main function
{
  // Add startup code here.
  
  //initial gripper position
  servo_angle(p1,1800);
  servo_angle(p2,900);
  
  //running other cogs
  cogstart(bt,NULL,stack3,sizeof(stack3));
  pause(2);  
  cogstart(right,NULL,stack2,sizeof(stack2));
  pause(2);
  cogstart(left,NULL,stack1,sizeof(stack1));
  pause(2);
  cogstart(gripper,NULL,stack4,sizeof(stack4));
  pause(2);

}

void left(){
  while(1){
    if(ls!=0){
       if(stop==1){ls=0; }
    high(12); 
    waitcnt(CNT+((1500-ls*2)*80));
    low(12);
    pause(20); 
  }    
  }    
}

void right(){
   while(1){
    if(rs!=0){
    if(stop==1){rs=0;}
     high(13);
    waitcnt(CNT+((1500-rs*2)*80));
    low(13);
    pause(20);    
  }    
  }   
}    

void bt(){
  fdserial *btSerial = fdserial_open(0, 1, 0, 9600);
  int p,side=-1,dir,speed;
  while(1)
  {
    if(fdserial_rxReady(btSerial) > 0) {
            // Read data from the Bluetooth module
            p = (int)(fdserial_rxChar(btSerial));
            if(p==255){stop=1; } //bluetooth command value  for stopping the motion
            else if(p==253){grip=1;} //bluetooth command value  for performing pickup 
            else if(p==254){grip=2;} //bluetooth command value  for performing drop 
            else if(p!=0){ //bluetooth command value  for decoding the motor speed commands
              stop=0;
              side= (p&1); //LSB representing which motor to be actuated
             dir = (p>>1)&1; //second LSB representing direction of rotation
             speed = 2*(p>>2); //rest of the bits in the data byte representing the motor speed 
             speed*=(-1+2*dir); 
             if(side==1){rs=speed;}
             else if(side==0){ls=-1*speed;}
            fdserial_rxFlush(btSerial);
            pause(10);
          }            
          }         
  }  
}   

void gripper(){
  while(1){
    switch(grip){
       case 1: // pickup
       move(p1,a1,1200);
       move(p2,a2,0);
       pause(5);
       move(p1,1200,1800);
       a1=1800; a2=0;
       grip=0;
       break;
       
       case 2://drop
       move(p1,a1,1200);
       move(p2,a2,900);
       move(p1,1200,1800);
       a1=1800; a2=900;
       grip=0;
      break;
    }    
  }         
}


//Function for making the servo motion smoother
void move(int p, int ang1, int ang2){
  int inc=10;
  if(ang2<ang1){
      inc=-10;
    }
    int a=ang1;
    while(a!=ang2){
      a+=inc;
      servo_angle(p,a);
      pause(1);
    }   
} 
  

