#include "list.h"
#include "list.c"

int main(void) {
	List list;
	initList(&list);
	
	insertFirstNode(&list, 4);		// [4]
	printList(&list);
	insertFirstNode(&list, 2);		// [2, 4]
	printList(&list);
	insertFirstNode(&list, 1);		// [1, 2, 4]
	printList(&list);
	
	insertNode(&list, 2, 3);		// [1, 2, 3, 4]
	printList(&list);
	
	deleteNode(&list, 2);			// [1, 3, 4]
	printList(&list);
	
	cleanupList(&list);
	return 0;
}
