/*
   l298n.c

   gcc -o l298n l298n.c -lpigpio -lrt -lpthread

   sudo ./l298n
*/

#include <stdio.h>
#include <pigpio.h>

int main(int argc, char *argv[])
{
   double start;

   if (gpioInitialise() < 0)
   {
      fprintf(stderr, "pigpio initialisation failed\n");
      return 1;
   }

   /* Set GPIO modes */
   gpioSetMode(12, PI_OUTPUT);
   gpioSetMode(21, PI_OUTPUT);
   gpioSetMode(20, PI_OUTPUT);
   gpioSetMode(23, PI_OUTPUT);
   gpioSetMode(24, PI_OUTPUT);
   gpioSetMode(13, PI_OUTPUT);

   gpioWrite(12, 0); /* on */
   gpioWrite(13, 0); /* on */

   /*
   start = time_time();

   while ((time_time() - start) < 60.0)
   {
      // motor on
      gpioWrite(12, 1); 
      gpioWrite(13, 1); 

      // forward
      gpioWrite(21, 1);
      gpioWrite(20, 0);
      gpioWrite(23, 1);
      gpioWrite(24, 0);

      time_sleep(3.5);

      // backward
      gpioWrite(21, 0);
      gpioWrite(20, 1);
      gpioWrite(23, 0);
      gpioWrite(24, 1);

      time_sleep(3.5);

      // motor stop
      gpioWrite(12, 0);
      gpioWrite(13, 0)

      time_sleep(3.5);
   }
   */

   start = time_time();

   while ((time_time() - start) < 60.0)
   {
      int x = (int)(time_time() - start)*3+75;
      gpioPWM(12, 0);
      gpioPWM(13, 0);

      // forward
      gpioWrite(21, 1);
      gpioWrite(20, 0);
      gpioWrite(23, 1);
      gpioWrite(24, 0);

      gpioPWM(12, x);
      gpioPWM(13, x);
      time_sleep(1);
   }

   // stop
   gpioWrite(12, 0);
   gpioWrite(13, 0);

   


   /* Stop DMA, release resources */
   gpioTerminate();

   return 0;
}
