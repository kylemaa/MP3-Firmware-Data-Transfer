#include "UART_DR.hpp"
#include <stdint.h>
#include <cstring>

using namespace std;


class LCD_D
{
public:
	LCD_D();
	void send_to_lcd(string data);
    void write(string column, string row, string data);
    void clear_lcd();
	void clear_line(string row);
};

UART_D uart_3;

LCD_D::LCD_D()
{
    uart_3.initialize(UART3);

    uart_3.transmit(UART3, 0xF0);
}

void LCD_D::send_to_lcd(string data)
{
    for(int i = 0; i < data.length(); i++){

        char send_data = data[i]; 
        uart_3.transmit(UART3, send_data);
        delay_ms(1);
    }

}

void LCD_D::write(string column, string row, string data)
{	
	string command = "$GOTO:";
	command += column;
	string colon = ":";
	command += colon;
	command += row;
	command += "\n";

	send_to_lcd(command);
	send_to_lcd(data);
}

void LCD_D::clear_lcd()
{
 	string clear = "$CLR_SCR\n";
 	send_to_lcd(clear);
    delay_ms(10);

}


void LCD_D::clear_line(string row)
{
	string cmd = "$CLR_LINE:";
	cmd += row; 
	cmd += "\n";
	send_to_lcd(cmd);
	delay_ms(10);
}