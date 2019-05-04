#include "AccelUNO.h"

AccelUNO A;

int data = 0;

void mpuPrint() {
  Serial.print("Ax : "); Serial.print(A.Ax()); Serial.print(" || ");
  Serial.print("Ay : "); Serial.print(A.Ay()); Serial.print(" || ");
  Serial.print("Az : "); Serial.print(A.Az()); Serial.print(" || ");
  Serial.print("Temp : "); Serial.print(A.MPUTemp()); Serial.print(" || ");
  Serial.print("Gx : "); Serial.print(A.Gx()); Serial.print(" || ");
  Serial.print("Gy : "); Serial.print(A.Gy()); Serial.print(" || ");
  Serial.print("Gz : "); Serial.println(A.Gz());
}

void setup() {
  Serial.begin(115200);
  //A.FindMPUAddress();
  //A.MPUAddressSet(0x68);
  A.BeginMPU();

}

void loop() {
  mpuPrint();
  delay(333);

}
