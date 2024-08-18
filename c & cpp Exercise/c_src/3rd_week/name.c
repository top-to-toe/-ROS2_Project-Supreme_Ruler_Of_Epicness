#include <stdio.h>

int main(void) {

	// char name[21];
	char buf[1024];
	char *name;
	name = buf;
		
	scanf("%20s", name);					// scanf("%s", &name[0]);

	printf("%s\n", name + 3);			// printf("%s\n", &name[0]);
	return 0;
}
