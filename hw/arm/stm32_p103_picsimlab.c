/*
 * Olimex STM32 P103 Development Board
 *
 * Copyright (C) 2010 Andre Beckus
 *
 * Implementation based on
 * Olimex "STM-P103 Development Board Users Manual Rev. A, April 2008"
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "hw/arm/stm32.h"
#include "hw/sysbus.h"
#include "hw/arm/arm.h"
#include "hw/devices.h"
#include "ui/console.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"

#ifndef WIN
#include<sys/types.h>
#include<sys/socket.h>
#include<sys/un.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#else
#include<winsock2.h>
#include<ws2tcpip.h>
WORD wVersionRequested = 2;
WSADATA wsaData;
#endif

typedef struct
{
 Stm32 *stm32;

 //bool last_button_pressed;
 qemu_irq pin_irq[49];
 qemu_irq *pout_irq;

 int connected;
 int sockfd;

 QemuThread thread;
 //QemuMutex dat_lock;

 DeviceState *gpio_a;
 DeviceState *gpio_b;
 DeviceState *gpio_c;
 DeviceState *gpio_d;
 DeviceState *uart1;
 DeviceState *uart2;
 DeviceState *uart3;
} Stm32P103;

static void
pout_irq_handler(void *opaque, int n, int level)
{
 Stm32P103 *s = (Stm32P103 *) opaque;
 char val;

 switch (level)
  {
  case 0:
   if (s->connected)
    {
     val = (0x7F & n);
     if (send (s->sockfd, &val, 1, 0) != 1)
      {
       printf ("send error : %s \n", strerror (errno));
       exit (1);
      }
    }
   break;
  case 1:
   if (s->connected)
    {
     val = (0x7F & n) | 0x80;
     if (send (s->sockfd, &val, 1, 0) != 1)
      {
       printf ("send error : %s \n", strerror (errno));
       exit (1);
      }
    }
   break;
  }
}
/*
static void
stm32_p103_key_event(void *opaque, int keycode)
{
 Stm32P103 *s = (Stm32P103 *) opaque;
 bool make;
 int core_keycode;

 if ((keycode & 0x80) == 0)
  {
   make = true;
   core_keycode = keycode;
  }
 else
  {
   make = false;
   core_keycode = keycode & 0x7f;
  }

 // Responds when a "B" key press is received.
 // Inside the monitor, you can type "sendkey b"
 
 if (core_keycode == 0x30)
  {
   if (make)
    {
     if (!s->last_button_pressed)
      {
       qemu_irq_raise (s->button_irq);
       s->last_button_pressed = true;
      }
    }
   else
    {
     if (s->last_button_pressed)
      {
       qemu_irq_lower (s->button_irq);
       s->last_button_pressed = false;
      }
    }
  }
 return;

}
*/

static void *
remote_gpio_thread(void * arg)
{
 Stm32P103 *s = (Stm32P103 *) arg;
 struct sockaddr_in serv;
 unsigned char buff;

 if ((s->sockfd = socket (PF_INET, SOCK_STREAM, 0)) < 0)
  {
   printf ("socket error : %s \n", strerror (errno));
   exit (1);
  }
 memset (&serv, 0, sizeof (serv));
 serv.sin_family = AF_INET;
 serv.sin_addr.s_addr = inet_addr ("127.0.0.1");
 serv.sin_port = htons (2200);

 while (connect (s->sockfd, (struct sockaddr *) & serv, sizeof (serv)) < 0)
  {
   printf ("connect error : %s \n", strerror (errno));
   sleep (1);
  }

 s->connected = 1;

  while (1)
  {

   if ((recv (s->sockfd, & buff, 1, 0)) > 0)
    {

     if (buff == 0x00)//EXIT
      {
       exit (-1);
      }
     else if (buff == 0xFF)//RESET
      {
       qemu_system_reset_request ();
      }
     else if (buff & 0x80)
      {
       qemu_irq_raise (s->pin_irq[buff & 0x7F]);
      }
     else
      {
       qemu_irq_lower (s->pin_irq[buff & 0x7F]);
      }
    }
  }

 return NULL;
}

static void
stm32_p103_picsimlab_init(MachineState *machine)
{
 const char* kernel_filename = machine->kernel_filename;
 qemu_irq *pout_irq;
 Stm32P103 *s;

 s = (Stm32P103 *) g_malloc0 (sizeof (Stm32P103));

 stm32_init (/*flash_size*/0x0001ffff,
             /*ram_size*/0x00004fff,
             kernel_filename,
             8000000,
             32768);

 s->gpio_a = DEVICE (object_resolve_path ("/machine/stm32/gpio[a]", NULL));
 s->gpio_b = DEVICE (object_resolve_path ("/machine/stm32/gpio[b]", NULL));
 s->gpio_c = DEVICE (object_resolve_path ("/machine/stm32/gpio[c]", NULL));
 s->gpio_d = DEVICE (object_resolve_path ("/machine/stm32/gpio[d]", NULL));
 s->uart1 = DEVICE (object_resolve_path ("/machine/stm32/uart[1]", NULL));
 s->uart2 = DEVICE (object_resolve_path ("/machine/stm32/uart[2]", NULL));
 s->uart3 = DEVICE (object_resolve_path ("/machine/stm32/uart[3]", NULL));
 assert (s->gpio_a);
 assert (s->gpio_b);
 assert (s->gpio_c);
 assert (s->gpio_d);
 assert (s->uart2);
 assert (s->uart1);
 assert (s->uart3);

 pout_irq = qemu_allocate_irqs (pout_irq_handler, s, 65);

 //0
 //1 VBAT
 qdev_connect_gpio_out (s->gpio_c, 13, pout_irq[2]); s->pin_irq[2] = qdev_get_gpio_in (s->gpio_c, 13);
 qdev_connect_gpio_out (s->gpio_c, 14, pout_irq[3]);  s->pin_irq[3] = qdev_get_gpio_in (s->gpio_c, 14);
 qdev_connect_gpio_out (s->gpio_c, 15, pout_irq[4]); s->pin_irq[4] = qdev_get_gpio_in (s->gpio_c, 15);
 qdev_connect_gpio_out (s->gpio_d, 0, pout_irq[5]); s->pin_irq[5] = qdev_get_gpio_in (s->gpio_d, 0);
 qdev_connect_gpio_out (s->gpio_d, 1, pout_irq[6]); s->pin_irq[6] = qdev_get_gpio_in (s->gpio_d, 1);
 //7 RST
 qdev_connect_gpio_out (s->gpio_c, 0, pout_irq[8]); s->pin_irq[8] = qdev_get_gpio_in (s->gpio_c, 0);
 qdev_connect_gpio_out (s->gpio_c, 1, pout_irq[9]); s->pin_irq[9] = qdev_get_gpio_in (s->gpio_c, 1);
 qdev_connect_gpio_out (s->gpio_c, 2, pout_irq[10]); s->pin_irq[10] = qdev_get_gpio_in (s->gpio_c, 2);
 qdev_connect_gpio_out (s->gpio_c, 3, pout_irq[11]); s->pin_irq[11] = qdev_get_gpio_in (s->gpio_c, 3);
 //12 VSSA
 //13 VDDA 
 qdev_connect_gpio_out (s->gpio_a, 0, pout_irq[14]); //TODO pin input irqs
 qdev_connect_gpio_out (s->gpio_a, 1, pout_irq[15]);
 qdev_connect_gpio_out (s->gpio_a, 2, pout_irq[16]);

 qdev_connect_gpio_out (s->gpio_a, 3, pout_irq[17]);
 //18 VSS
 //19 VDD
 qdev_connect_gpio_out (s->gpio_a, 4, pout_irq[20]);
 qdev_connect_gpio_out (s->gpio_a, 5, pout_irq[21]);
 qdev_connect_gpio_out (s->gpio_a, 6, pout_irq[22]);
 qdev_connect_gpio_out (s->gpio_a, 7, pout_irq[23]);
 qdev_connect_gpio_out (s->gpio_c, 4, pout_irq[24]);
 qdev_connect_gpio_out (s->gpio_c, 5, pout_irq[25]);
 qdev_connect_gpio_out (s->gpio_b, 0, pout_irq[26]);
 qdev_connect_gpio_out (s->gpio_b, 1, pout_irq[27]);
 qdev_connect_gpio_out (s->gpio_b, 2, pout_irq[28]);
 qdev_connect_gpio_out (s->gpio_b, 10, pout_irq[29]);
 qdev_connect_gpio_out (s->gpio_b, 11, pout_irq[30]);
 //31 VSS
 //32 VDD

 qdev_connect_gpio_out (s->gpio_b, 12, pout_irq[33]);
 qdev_connect_gpio_out (s->gpio_b, 13, pout_irq[34]);
 qdev_connect_gpio_out (s->gpio_b, 14, pout_irq[35]);
 qdev_connect_gpio_out (s->gpio_b, 15, pout_irq[36]);
 qdev_connect_gpio_out (s->gpio_c, 6, pout_irq[37]);
 qdev_connect_gpio_out (s->gpio_c, 7, pout_irq[38]);
 qdev_connect_gpio_out (s->gpio_c, 8, pout_irq[39]);
 qdev_connect_gpio_out (s->gpio_c, 9, pout_irq[40]);
 qdev_connect_gpio_out (s->gpio_a, 8, pout_irq[41]);
 qdev_connect_gpio_out (s->gpio_a, 9, pout_irq[42]);
 qdev_connect_gpio_out (s->gpio_a, 10, pout_irq[43]);
 qdev_connect_gpio_out (s->gpio_a, 11, pout_irq[44]);
 qdev_connect_gpio_out (s->gpio_a, 12, pout_irq[45]);
 qdev_connect_gpio_out (s->gpio_a, 13, pout_irq[46]);
 //47 VSS
 //48 VDD

 qdev_connect_gpio_out (s->gpio_a, 14, pout_irq[49]);
 qdev_connect_gpio_out (s->gpio_a, 15, pout_irq[50]);
 qdev_connect_gpio_out (s->gpio_c, 10, pout_irq[51]);
 qdev_connect_gpio_out (s->gpio_c, 11, pout_irq[52]);
 qdev_connect_gpio_out (s->gpio_c, 12, pout_irq[53]);
 qdev_connect_gpio_out (s->gpio_d, 2, pout_irq[54]);
 qdev_connect_gpio_out (s->gpio_b, 3, pout_irq[55]);
 qdev_connect_gpio_out (s->gpio_b, 4, pout_irq[56]);
 qdev_connect_gpio_out (s->gpio_b, 5, pout_irq[57]);
 qdev_connect_gpio_out (s->gpio_b, 6, pout_irq[58]);
 qdev_connect_gpio_out (s->gpio_b, 7, pout_irq[59]);
 //60 BOOT0
 qdev_connect_gpio_out (s->gpio_b, 8, pout_irq[61]);
 qdev_connect_gpio_out (s->gpio_b, 9, pout_irq[62]);
 //63 VSS
 //64 VDD

 /* Connect button to GPIO A pin 0 */
 //s->button_irq = qdev_get_gpio_in (s->gpio_a, 0);
 //qemu_add_kbd_event_handler (stm32_p103_key_event, s);

 /* Connect RS232 to UART */
 stm32_uart_connect (
                     (Stm32Uart *) s->uart2,
                     serial_hds[0],
                     STM32_USART2_NO_REMAP);

 /* These additional UARTs have not been tested yet... */
 stm32_uart_connect (
                     (Stm32Uart *) s->uart1,
                     serial_hds[1],
                     STM32_USART1_NO_REMAP);

 stm32_uart_connect (
                     (Stm32Uart *) s->uart3,
                     serial_hds[2],
                     STM32_USART3_NO_REMAP);

 //qemu_mutex_init (&s->dat_lock);
 qemu_thread_create (&s->thread, "remote_gpio", remote_gpio_thread, s, QEMU_THREAD_JOINABLE);

}

static QEMUMachine stm32_p103_picsimlab_machine = {
 .name = "stm32-p103-picsimlab",
 .desc = "Olimex STM32 p103 Dev Board (PICSimLab)",
 .init = stm32_p103_picsimlab_init,
};

static void
stm32_p103_picsimlab_machine_init(void)
{
 qemu_register_machine (&stm32_p103_picsimlab_machine);
}

machine_init(stm32_p103_picsimlab_machine_init);
