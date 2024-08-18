#include <stdio.h>
#include "string.h"

/* unsigned */ int my_strlen(const char *s) {

	int i;
	for(i = 0; s[i] != '\0'; ++i) {
		// 
	}
	
	return i;
}

void my_strcpy(char *des, const char *src) {
	int i;
	for(i = 0; src[i] != '\0'; ++i) {
		des[i] = src[i];
	}
	des[i] = '\0';
}

/* unsigned */ int my_strcmp(const char *s1, const char *s2) {
	int i;
	for(i = 0; s1[i] != '\0' && s2[i] != '\0' && s1[i] != s2[i]; ++i) {
		if(s1[i] < s2[i])
			return -1;
		else if(s1[i] > s2[i])
			return 1;
	}
	return s1[i] - s2[i];
}

void my_strcat(char *des, const char *src) {
	int i;
	for(i = 0; des[i] != '\0'; ++i) {
		//	
	}
	
	int j;
	for(j = 0; src[j] != '\0'; ++j) {
		des[i + j] = src[j];
	}
	des[i + j] = '\0';
}
