#include <Arduino.h>
#include "Behaviors.h"

Behaviors parkour;

void setup() {
  parkour.Init();
}

void loop() {
  parkour.Run();
}