#ifndef _H_LED_
#define _H_LED_

#include <Arduino.h>

class LED{
	private:
		int PIN;
		bool state;
	public:
		LED(int ptr_pin);
		void init();
		void light();
		void off();
		void toggle();
};
//publicは外からアクセスできる。クラスの中でしか使えない。クラスの外からはクラス名.でアクセスできる。変数も同じ

#endif