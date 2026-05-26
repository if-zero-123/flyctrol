#include "telemetry.h"

#include "debug_uart.h"

void Telemetry_PrintBoot(void)
{
  DebugUart_WriteLine("");
  DebugUart_WriteLine("NAZE32 custom firmware boot");
  DebugUart_WriteLine("CLI ready: type help");
}
