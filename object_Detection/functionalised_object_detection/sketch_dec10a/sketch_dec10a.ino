void setup() {
  // put your setup code here, to run once:
pinMode(LED_BUILTIN, OUTPUT);
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
double distanceBlue, distanceStripy,distanceYellow, distanceGreen,partialActivationBoundary,fullActivationBoundary;
double tempBlue, tempStripy, tempYellow, tempGreen;
double voltageBlue, voltageStripy, voltageYellow, voltageGreen;
double activateLeft, activateRight;
fullActivationBoundary = 15;//must be greater than or equal to 10
partialActivationBoundary = 75;//must be less than or equal to 80

tempBlue = analogRead(0);
voltageBlue = tempBlue * 5 / 1024;
distanceBlue = distanceBlueSensor(voltageBlue);

tempStripy = analogRead(1);
voltageStripy = tempStripy * 5 / 1024;
distanceStripy = distanceStripySensor(voltageStripy);

tempYellow = analogRead(2);
voltageYellow = tempYellow * 5 / 1024;
distanceYellow = distanceYellowSensor(voltageYellow);

tempGreen = analogRead(3);
voltageGreen = tempGreen * 5 / 1024;
distanceGreen = distanceGreenSensor(voltageGreen);

activateLeft = activationLeft(fullActivationBoundary, partialActivationBoundary, distanceYellow, distanceGreen);

activationRight(fullActivationBoundary, partialActivationBoundary, distanceStripy, distanceBlue);


delay(1000);
}

double activationLeft(double fullActivationBoundary, double partialActivationBoundary, double distanceYellow, double distanceGreen)
{
  double activateLeft;
  if ( (distanceYellow<=fullActivationBoundary) || (distanceGreen <=fullActivationBoundary))
  {
    activateLeft = 1.0;
  }

  if ( ((distanceYellow>=fullActivationBoundary) && (distanceYellow<=partialActivationBoundary)) || ((distanceGreen>=fullActivationBoundary) && (distanceGreen<=partialActivationBoundary)))
  {
    if(distanceYellow<distanceGreen)
    {
      activateLeft = linearRelationship(distanceYellow,partialActivationBoundary,fullActivationBoundary);
    }
    if(distanceYellow>distanceGreen)
    {
      activateLeft = linearRelationship(distanceGreen,partialActivationBoundary,fullActivationBoundary);
    }
  }

  return activateLeft;
  
}

double activationRight(double fullActivationBoundary, double partialActivationBoundary, double distanceStripy, double distanceBlue)
{
  double activateRight;
  
  if ( (distanceStripy<=fullActivationBoundary) || (distanceBlue <=fullActivationBoundary))
  {
      activateRight = 1.0;
  }

  if ( ((distanceStripy>=fullActivationBoundary) && (distanceStripy<=partialActivationBoundary)) || ((distanceBlue>=fullActivationBoundary) && (distanceBlue<=partialActivationBoundary)))
  {
    if(distanceStripy<distanceBlue)
    {
      activateRight = linearRelationship(distanceStripy,partialActivationBoundary,fullActivationBoundary);
    }
    if(distanceStripy>distanceBlue)
    {
      activateRight = linearRelationship(distanceBlue,partialActivationBoundary,fullActivationBoundary);
    }
  }

  return activateRight;

}

double linearRelationship(double distance,double x1, double x2)
{
  double percentActivation = 1-((distance - x2)/(x1-x2));
  return percentActivation;
}

double quadraticRelationship(double distance,double x1, double x2)
{
  double percentActivation = pow((1-((distance - x2)/(x1-x2))),2);
  return percentActivation;
}

double sqrtRelationship(double distance,double x1, double x2)
{
  double percentActivation = sqrt(1-((distance - x2)/(x1-x2)));
  return percentActivation;
}

double distanceBlueSensor(double voltage)
{
  double e = 2.718;
  double distance = 547.5 * pow(e,(-4.451*voltage)) + 29.19 * pow(e,(-0.4582*voltage));
  if(distance<10)
  {
    return 0;
  }
  if(distance>80)
  {
    return distance = 100;
  }
  else
  {
    return distance;
  }
} 

double distanceStripySensor(double voltage)
{
  double e = 2.718;
  double distance = 223.6 * pow(e,(-4.424*voltage)) + 41.66 * pow(e,(-0.6203*voltage));
  if(distance<10)
  {
    return 0;
  }
  if(distance>80)
  {
    return distance = 100;
  }
  else
  {
    return distance;
  }
} 

double distanceYellowSensor(double voltage)
{
  double e = 2.718;
  double distance = 462.4 * pow(e,(-5.733*voltage)) + 48.76 * pow(e,(-0.7032*voltage));
  if(distance<10)
  {
    return 0;
  }
  if(distance>80)
  {
    return distance = 100;
  }
  else
  {
    return distance;
  }
} 

double distanceGreenSensor(double voltage)
{
  double e = 2.718;
  double distance = 228.2 * pow(e,(-4.4*voltage)) + 38.9 * pow(e,(-0.584*voltage));
  if(distance<10)
  {
    return 0;
  }
  if(distance>80)
  {
    return distance = 100;
  }
  else
  {
    return distance;
  }
} 

