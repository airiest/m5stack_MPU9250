import processing.serial.*;

PFont font;
Serial port;

float roll_mad = 0;
float pitch_mad = 0;
float yaw_mad = 0;
float roll_dmp = 0;
float pitch_dmp = 0;
float yaw_dmp = 0;

void setup() {
  size(800, 500, P3D);
  frameRate(60);
  
  String[] ports = Serial.list();
  
  PFont font = createFont("Arial", 30);
  textFont(font);
  textSize(24);
  
  for (int i = 0; i < ports.length; i++) {
    println(i + ": " + ports[i]);
  }
  
  port = new Serial(this, ports[1], 115200);
}

void draw() {
  if (port.available() == 0) return;

  String str = port.readStringUntil('\n');
  if (str == null) return;

  String token[] = split(trim(str), ",");

  if (token.length == 3) {
    roll_mad = float(token[0]);
    pitch_mad = -float(token[1]);
    yaw_mad = 180 - float(token[2]);
  }
  else if (token.length == 4) {
    float euler[];
    euler = quaternion_to_euler_angle(float(token[0]), float(token[1]), float(token[2]), float(token[3]));
    roll_dmp = euler[0];
    pitch_dmp = euler[1];
    yaw_dmp = euler[2];
  }
  else
  {
    return;
  }
  
  background(0);
  fill(255, 255, 255);
  text("Madgwick Filter", width/2-270, height-80);
  text("Digital Motion Processing (DMP)", width/2+20, height-80);

  pushMatrix();
  draw_box(width/2-180, height/2, 0, 0, 150, 0, roll_mad, pitch_mad, yaw_mad);
  draw_box(width/2+180, height/2, 0, 150, 0, 0, roll_dmp, pitch_dmp, yaw_dmp);
  popMatrix();
}

float[] quaternion_to_euler_angle(float q0, float q1, float q2, float q3) {
  float q0q0 = q0 * q0;
  float q0q1 = q0 * q1;
  float q0q2 = q0 * q2;
  float q0q3 = q0 * q3;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q3q3 = q3 * q3;
  float roll = atan2(2.0 * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3);
  float pitch = asin(2.0 * (q0q2 - q1q3));
  float yaw = atan2(2.0 * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3);
  
  float[] euler = new float[3];
  euler[0] = roll * 180 /PI;
  euler[1] = -pitch * 180 /PI;
  euler[2] = -yaw  * 180 /PI;
  return euler;
}

void draw_box(int position_x, int position_y, int position_z, int color_r, int color_g, int color_b, float roll, float pitch, float yaw)
{
  pushMatrix();
  fill(color_r, color_g, color_b);
  translate(position_x, position_y, position_z);
  rotateY(-radians(yaw));
  rotateZ(-radians(pitch));
  rotateX(-radians(roll));
  box(150, 50, 150);
  popMatrix();
}
