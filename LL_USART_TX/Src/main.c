#include "main.h"

int main(){
	uart1_init();
	while(1){
		uart1_write('Y');
		for (int itr=0;itr<900000;itr++){};
	}
	return 0;
}

