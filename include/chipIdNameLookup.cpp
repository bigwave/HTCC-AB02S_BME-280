#include <Arduino.h>
#include "version.h"

void print_uint64_t(uint64_t num)
{

  char rev[128];
  char *p = rev + 1;

  while (num > 0)
  {
    *p++ = '0' + (num % 10);
    num /= 10;
  }
  p--;
  /*Print the number which is now in reverse*/
  while (p > rev)
  {
    Serial.print(*p--);
  }
}
String chipIdNameLookup()
{

  uint64_t chipID = getID();
  uint32_t low = (uint32_t)chipID;
  uint32_t high = (uint32_t)(chipID >> 32);
  Serial.printf("ChipID: %08X%08X  :  ", high, low);

  print_uint64_t(chipID);
  Serial.println();
  String name;
  switch (chipID)
  {
      case 0x000063A59B531B20LL: // 
        name = "htcc-ab02s-01";
        break;
      case 0x000093A3A6A70C29LL: // 
        name = "htcc-ab02s-02";
        break;
      case 0x000073A3A6A73519LL: // 
        name = "htcc-ab02s-03";
        break;
      case 0x000043A3A6A71A2FLL: // 
        name = "htcc-ab02s-04";
        break;


  default:
    name = "Unknown Device!!!!";
    break;
  }

  Serial.println(name);
  return name;
}

