// Tento kód číta dáta z ultrazvukového senzora a publikuje ich na ROS topicu "ultrasonic_range".
#include <ros.h>           // Táto knižnica zabezpečuje komunikáciu s ROS 
#include <std_msgs/Float32.h>  // Použijeme jednoduchú ROS správu typu Float32 pre vzdialenosť

// Deklarácia ROS uzla
ros::NodeHandle nh;

// Vytvorenie ROS správy a publishera
std_msgs::Float32 distance_msg;
ros::Publisher distance_pub("ultrasonic_range", &distance_msg);

// Definuj piny, ku ktorým je pripojený ultrazvukový senzor (napr. HC-SR04)
const int trigPin = 3;
const int echoPin = 2;

void setup() {
  // Nastavenie pinov
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Inicializácia ROS uzla
  nh.initNode();
  nh.advertise(distance_pub);
  nh.getHardware()->setBaud(57600); 
}

void loop() {
  // Vytvorenie pulzu na trig pinu
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Meranie trvania pulzu na echo pine
  long duration = pulseIn(echoPin, HIGH);
  
  // Výpočet vzdialenosti v centimetroch (rýchlosť zvuku ~0.034 cm/us a delíme dvoma, pretože ide o tam-späť cestu)
  float distance = duration * 0.034 / 2;

  // Aktualizácia správy a publikovanie na ROS topic
  distance_msg.data = distance;
  distance_pub.publish(&distance_msg);
  
  // Aktualizácia ROS komunikácie
  nh.spinOnce();
  
  // Krátka pauza (100 ms, teda frekvencia približne 10 Hz)
  delay(100);
}

