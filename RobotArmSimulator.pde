import javax.vecmath.Vector3f;
import javax.vecmath.Quat4f;
import com.bulletphysics.linearmath.Transform;
import peasy.*;
import bRigid.*;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

PeasyCam cam;

BPhysics physics;
BPlane ground;
Vector3f groundSize = new Vector3f(1000f, 1000f, 0.01f);
float[] groundColor = {127f, 127f, 127f};
Vector3f[] linkSize = {
  new Vector3f(0.2f, 0.2f, 0.1f), 
  new Vector3f(0.1f, 0.1f, 0.5f), 
  new Vector3f(0.1f, 0.1f, 0.5f), 
  new Vector3f(0.1f, 0.1f, 0.1f)
};
Vector3f[] linkPos = {
  new Vector3f(0.0f, 0.0f, 0.05f), 
  new Vector3f(0.0f, 0.0f, 0.35f), 
  new Vector3f(0.0f, 0.0f, 0.85f), 
  new Vector3f(0.0f, 0.0f, 1.15f)
};
Vector3f baseSize = new Vector3f(10.0f, 10.0f, 0.01f);

float[] linkMass = {5.0f, 0.5f, 0.5f, 0.1f};

BBox base;
BBox[] links = new BBox[4];
BJointHinge[] linkJoints = new BJointHinge[4];
BBox hand;
BBox[] finger = new BBox[2];
BJointSlider[] fingerJoints = new BJointSlider[2];
float fingerInitPos = 0.15f;
Vector3f[] fingerSize = {
  new Vector3f(0.3f, 0.02f, 0.1f), 
  new Vector3f(0.3f, 0.02f, 0.1f)
};

Vector3f[] fingerPos = {
  new Vector3f(0.0f, fingerInitPos, 1.25f), 
  new Vector3f(0.0f, -fingerInitPos, 1.25f)
};
float fingerMass = 0.01;

float[] partsColor = {255f, 0f, 0f};
//Vector3f partsSize = new Vector3f(0.05, 0.05, 0.05);
Vector3f partsPos = new Vector3f(0.7, 0.4, 0.05);
float partsMass = 0.1;
//BBox parts;
float partsRadius = 0.04;
BSphere parts;

int count = 0;
int maxCount = 200;
float[] initAngle = {0f, 0f, 0f, 0f};

float[] lastAngle = new float[4];
Vector3f lastObjectPos = new Vector3f();

PrintWriter output = createWriter("test.txt");

Vector3f quaternion_to_euler_angle(Quat4f quat) {
  float ysqr = quat.y * quat.y;

  float t0 = 2.0 * (quat.w * quat.x + quat.y * quat.z);
  float t1 = 1.0 - 2.0 * (quat.x * quat.x + ysqr);
  float X = atan2(t0, t1);

  float t2 = 2.0 * (quat.w * quat.y - quat.z * quat.x);
  t2 = (t2 > 1.0)? 1.0:t2;
  t2 = (t2 < -1.0)? -1.0:t2;
  float Y = asin(t2);

  float t3 = 2.0 * (quat.w * quat.z + quat.x * quat.y);
  float t4 = 1.0 - 2.0 * (ysqr + quat.z * quat.z);

  float Z = atan2(t3, t4);

  Vector3f forReturn = new Vector3f(X, Y, Z);

  return forReturn;
}

private enum ArmStatus {
    Move, 
    Catch, 
    Pickup, 
    Release;
}

ArmStatus Status = ArmStatus.Move;

public void setup() {
  //size(1280, 720, P3D);
  size(400, 400, P3D);
  frameRate(60);

  float fov = PI/3.0;
  float cameraZ = (height/2.0) / tan(fov/2.0);
  float nearClippingDistance = 0.01; // default is cameraZ/10.0
  perspective(fov, float(width)/float(height), nearClippingDistance, cameraZ*10.0);

  cam = new PeasyCam(this, 0, 1, 0.5, 1);
  cam.rotateX(-HALF_PI+.2f);
  cam.setMinimumDistance(0.01);
  cam.setMaximumDistance(40);



  physics = new BPhysics();
  physics.world.setGravity(new Vector3f(0, 0, -9.8));

  ground = new BPlane(new Vector3f(0, 0, -1), new Vector3f(0, 0, 1));
  physics.addPlane(ground);

  base = new BBox(this, 0, new Vector3f(0, 0, 0), baseSize, false);
  physics.addBody(base);



  for (int i = 0; i < links.length; i++) {
    links[i] = new BBox(this, linkMass[i], linkPos[i], linkSize[i], true);
    //links[i] = new BBox(this, mass, linkPos[i].x, linkPos[i].y, linkPos[i].z, linkSize[i].x, linkSize[i].y, linkSize[i].z);
    //links[i].rigidBody.setAngularFactor(1.1f);
    //links[i].setPosition(linkPos[i].x, linkPos[i].y, linkPos[i].z);
    physics.addBody(links[i]);

    if (i==0)
    {

      Vector3f pivA = new Vector3f(0, 0, 0);
      Vector3f pivB = new Vector3f(0, 0, -linkSize[i].z/2.0);
      Vector3f axisInA = new Vector3f(0, 0, -1);
      Vector3f axisInB = new Vector3f(0, 0, -1);
      linkJoints[i] = new BJointHinge(base, links[i], pivA, pivB, axisInA, axisInB);
    } else
    {
      Vector3f pivA = new Vector3f(0, 0, linkSize[i-1].z/2.0);
      Vector3f pivB = new Vector3f(0, 0, -linkSize[i].z/2.0);
      Vector3f axisInA = new Vector3f(0, 1, 0);
      Vector3f axisInB = new Vector3f(0, 1, 0);
      linkJoints[i] = new BJointHinge(links[i-1], links[i], pivA, pivB, axisInA, axisInB);
    }
    physics.addJoint(linkJoints[i]);


    //linkJoints[i].enableAngularMotor(true, 1, 1f);
  }



  for (int i = 0; i < finger.length; i++) {
    finger[i] = new BBox(this, fingerMass, fingerPos[i], fingerSize[i], true);
    physics.addBody(finger[i]);
    Vector3f pivA = new Vector3f(fingerPos[i].x, fingerPos[i].y, linkSize[linkSize.length-1].z/2.0);
    Vector3f pivB = new Vector3f(0, 0, -fingerSize[i].z/2.0);
    Transform localA = new Transform();
    Transform localB = new Transform();
    localA.setIdentity();
    localB.setIdentity();
    localA.origin.set(pivA);
    localB.origin.set(pivB);

    localA.setRotation(new Quat4f(1, 0, 0, 1));
    //localA.setRotation(new Quat4f(cos(PI/2f),0,0,sin(PI/2f)));
    //localB.setRotation(new Quat4f(cos(PI/2f),0,0,sin(PI/2f)));
    //localA.setRotation(new Quat4f(cos(PI/4f),0,sin(PI/4f),0));
    //localB.setRotation(new Quat4f(cos(PI/4f),0,sin(PI/4f),0));
    localA.setRotation(new Quat4f(cos(PI/4f), sin(PI/4f), 0, 0));
    localB.setRotation(new Quat4f(cos(PI/4f), sin(PI/4f), 0, 0));
    fingerJoints[i] = new BJointSlider(links[links.length-1], finger[i], localA, localB, false);
    //fingerJoints[i].setUpperLinLimit(0.1);
    //fingerJoints[i].setLowerLinLimit(0);
    //fingerJoints[i].setLowerAngLimit(0);
    //fingerJoints[i].setUpperAngLimit(0);
    physics.addJoint(fingerJoints[i]);
  }
  fingerJoints[0].setUpperLinLimit(0.1);
  fingerJoints[0].setLowerLinLimit(0);
  fingerJoints[1].setUpperLinLimit(0);
  fingerJoints[1].setLowerLinLimit(-0.1);
  fingerJoints[0].setUpperAngLimit(0);
  fingerJoints[0].setLowerAngLimit(0);
  fingerJoints[1].setUpperAngLimit(0);
  fingerJoints[1].setLowerAngLimit(0);
  fingerJoints[0].setPoweredLinMotor(true);
  fingerJoints[1].setPoweredLinMotor(true);
  fingerJoints[0].setPoweredAngMotor(false);
  fingerJoints[1].setPoweredAngMotor(false);

  //parts = new BBox(this, partsMass, partsPos, partsSize, true);
  parts = new BSphere(this, partsMass, partsPos.x, partsPos.y, partsPos.z, partsRadius);
  physics.addBody(parts);

  //thread("SubThread");
}

private void drawGround()
{
  specular(groundColor[0], groundColor[1], groundColor[2]);
  pushMatrix();
  Vector3f pos = ground.rigidBody.getCenterOfMassPosition(new Vector3f());
  Quat4f rot = ground.rigidBody.getOrientation(new Quat4f());
  Vector3f euler = quaternion_to_euler_angle(rot);
  //println(pos);
  translate(pos.x, pos.y, pos.z);
  rotateZ(euler.z);
  rotateY(euler.y);
  rotateX(euler.x);

  box(groundSize.x, groundSize.y, groundSize.z);
  popMatrix();
}

private void drawLinks()
{
  for (int i = 0; i < links.length; i++) {
    Vector3f pos = links[i].rigidBody.getCenterOfMassPosition(new Vector3f());
    Quat4f rot = links[i].rigidBody.getOrientation(new Quat4f());
    Vector3f euler = quaternion_to_euler_angle(rot);
    Vector3f size = links[i].getShapeScale();

    pushMatrix();


    translate(pos.x, pos.y, pos.z);
    rotateZ(euler.z);
    rotateY(euler.y);
    rotateX(euler.x);

    box(linkSize[i].x, linkSize[i].y, linkSize[i].z);

    popMatrix();
  }
  /*
  pushMatrix();
   translate(0, 0, -1);
   box(1, 1, 0.01);
   popMatrix();
   */
}

private void drawFinger()
{
  for (int i = 0; i < finger.length; i++) {
    Vector3f pos = finger[i].rigidBody.getCenterOfMassPosition(new Vector3f());
    Quat4f rot = finger[i].rigidBody.getOrientation(new Quat4f());
    Vector3f euler = quaternion_to_euler_angle(rot);
    Vector3f size = finger[i].getShapeScale();

    pushMatrix();


    translate(pos.x, pos.y, pos.z);
    rotateZ(euler.z);
    rotateY(euler.y);
    rotateX(euler.x);

    box(fingerSize[i].x, fingerSize[i].y, fingerSize[i].z);

    popMatrix();
  }
}

private void drawParts()
{
  specular(partsColor[0], partsColor[1], partsColor[2]);
  pushMatrix();
  Vector3f pos = parts.rigidBody.getCenterOfMassPosition(new Vector3f());
  Quat4f rot = parts.rigidBody.getOrientation(new Quat4f());
  Vector3f euler = quaternion_to_euler_angle(rot);
  //println(pos);
  translate(pos.x, pos.y, pos.z);
  rotateZ(euler.z);
  rotateY(euler.y);
  rotateX(euler.x);
  sphere(partsRadius);
  //box(partsSize.x, partsSize.y, partsSize.z);
  popMatrix();
}

private void moveOnObject() {
  float handHeight = 0.12;
  Vector3f partsCurrentPos = parts.rigidBody.getCenterOfMassPosition(new Vector3f());
  Vector3f targetPos = new Vector3f(partsCurrentPos.x, partsCurrentPos.y, partsCurrentPos.z + handHeight);
  float pos = ((float)count / (float)maxCount);
  controlPos(targetPos, pos);
  if (count < maxCount)
  {
    count += 1;
  }
  else
  {
    Status = ArmStatus.Catch;
    count = 0;
    lastObjectPos = targetPos;
  }
  controlFinger(false);
}

private void catchObject() {
  Vector3f targetPos = lastObjectPos;
  float pos = 1.0f;
  controlPos(targetPos, pos);
  if (count < maxCount)
  {
    count += 1;
  }
  else
  {
    Status = ArmStatus.Catch;
  }
  controlFinger(true);
}

private void controlPos(Vector3f targetPos, float pos) {

  

  float[] currentAngle = new float[4];
  for (int i = 0; i < currentAngle.length; i++) {
    currentAngle[i] = linkJoints[i].getHingeAngle();
  }

  if (count == 0)
  {
    for (int i = 0; i < initAngle.length; i++) {
      initAngle[i] = currentAngle[i];
    }
  }

  
  float[] targetAngle = new float[4];
  targetAngle[0] = atan2(targetPos.y, targetPos.x);
  //println(targetPos.y+":"+targetPos.x+":"+targetAngle[0]);
  
  //Vector3f link1CurrentPos = links[1].rigidBody.getCenterOfMassPosition(new Vector3f());
  
  //println(link1CurrentPos.y+":"+link1CurrentPos.x+":"+atan2(link1CurrentPos.y, link1CurrentPos.x));


  float l1 = linkSize[1].z;
  float l2 = linkSize[2].z;
  float px = sqrt(pow(targetPos.x, 2)+pow(targetPos.y, 2));
  float py = targetPos.z;

  float C2 = (pow(px, 2)+pow(py, 2)-pow(l1, 2)-pow(l2, 2))
    / (2*l1*l2);
  targetAngle[2] = -acos(C2);



  float S2 = sin(targetAngle[2]);

  targetAngle[1] = atan2(-l2*S2*px + (l1+l2*C2)*py, 
    (l1+l2*C2)*px + l2*S2*py);
  targetAngle[1] = targetAngle[1] - PI/2f;




  targetAngle[3] = -PI - targetAngle[1] - targetAngle[2];

  

  float[] targetAngle_t = new float[4];
  for (int i = 0; i < targetAngle_t.length; i++) {
    targetAngle_t[i] = initAngle[i] + (targetAngle[i] - initAngle[i]) * pos;
  }


  controlJoint(targetAngle_t[0], targetAngle_t[1], targetAngle_t[2], targetAngle_t[3]);

  

  //println(l1+":"+l2+":"+px+":"+py+":"+S2+":"+C2);
  //println(targetAngle0+":"+targetAngle1+":"+targetAngle2 + ":" + targetAngle3 +":"+currentAngle0+":"+currentAngle1+":"+currentAngle2 + ":" + currentAngle3);
}

void controlJoint(float targetAngle0, float targetAngle1, float targetAngle2, float targetAngle3)
{
  float[] k = {30.0, 30.0, 30.0, 30.0};
  float kd = 0;
  float[] torque = {10.0, 1.0, 1.0, 1.0};

  float[] targetAngle = new float[4];
  targetAngle[0] = targetAngle0;
  targetAngle[1] = targetAngle1;
  targetAngle[2] = targetAngle2;
  targetAngle[3] = targetAngle3;

  float[] currentAngle = new float[4];
  for (int i = 0; i < currentAngle.length; i++) {
    currentAngle[i] = linkJoints[i].getHingeAngle();
  }

  for (int i = 0; i < currentAngle.length; i++) {
    //print(currentAngle[i]+"\t"+targetAngle[i]+"\t");
    output.print(currentAngle[i]+"\t"+targetAngle[i]+"\t");
  }
  //print("\n");
  output.print("\n");
  output.flush();


  if (count == 0)
  {
    for (int i = 0; i < lastAngle.length; i++) {
      lastAngle[i] = currentAngle[i];
    }
  }


  for (int i = 0; i < linkJoints.length; i++) {
    float vel = k[i] * (targetAngle[i] - currentAngle[i]) - kd * (currentAngle[i] - lastAngle[i]);
    linkJoints[i].enableAngularMotor(true, vel, torque[i]);
  }
  //linkJoints[0].enableMotor(true);
  //linkJoints[0].setMotorTargetVelocity(vel0);
  //linkJoints[0].setMotorTarget(0.1,0.1);
  //println(linkJoints[0].getLimitSoftness());

  for (int i = 0; i < lastAngle.length; i++) {
    lastAngle[i] = currentAngle[i];
  }
}


void controlFinger(boolean on)
{
  float k = 10.0;
  for (int i = 0; i < finger.length; i++) 
  {
    float targetPos = 0;
    fingerJoints[i].setMaxLinMotorForce(10);
    float currentPos = fingerJoints[i].getLinearPos();

    if (!on)
    {
      targetPos = 0;
    } else
    {
      if (i == 0)
      {
        targetPos = (fingerInitPos);
      } else if (i == 1)
      {
        targetPos = -(fingerInitPos);
      }
    }
    float vel = k * (targetPos - currentPos);
    fingerJoints[i].setTargetLinMotorVelocity(vel);
  }
}
/*
void SubThread() {
 while (true)
 {
 controlPos();
 if (count < maxCount)
 {
 controlFinger(false);
 } else
 {
 controlFinger(true);
 }
 physics.update();
 try {
 Thread.sleep(1);
 } 
 catch(InterruptedException ex) {
 ex.printStackTrace();
 }
 }
 }
 */

public void draw() {
  //background(255);

  background(0);
  ambientLight(20, 20, 20);    //環境光を当てる
  lightSpecular(255, 255, 255);    //光の鏡面反射色（ハイライト）を設定
  directionalLight(100, 100, 100, 0, 1, -1);    //指向性ライトを設定
  //lights();
  /*
  camera(0.0, 10.0, 10.0, // 視点X, 視点Y, 視点Z
   0.0, 0.0, -10.0, // 中心点X, 中心点Y, 中心点Z
   0.0, 0.0, -1.0); // 天地X, 天地Y, 天地Z
   */

  //controlPos();
  if (Status == ArmStatus.Move)
  {
    moveOnObject();
    controlFinger(false);
  } else if(Status == ArmStatus.Catch)
  {
    catchObject();
    controlFinger(true);
  }
  physics.update();
  //count++;
  //controlJoint(0, 0, 0, 0);
  /*
  fingerJoints[0].setMaxLinMotorForce(1);
   fingerJoints[1].setMaxLinMotorForce(1);
   fingerJoints[0].setTargetLinMotorVelocity(-2);
   fingerJoints[1].setTargetLinMotorVelocity(2);
   
   println(fingerJoints[0].getLinearPos()+":"+fingerJoints[1].getLinearPos());
   */

  //physics.display(240, 240, 240);

  //Vector3f dist = links[0].rigidBody.getCenterOfMassPosition(new Vector3f());
  //println(dist);


  
  drawGround();
  drawLinks();
  drawFinger();
  drawParts();
}
