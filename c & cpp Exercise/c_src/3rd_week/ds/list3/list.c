#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "list.h"

static Node *createNode(int data, Node *next) {
	Node *p = malloc(sizeof(Node));
	assert(p );
	p->data = data;
	p->next = next;
	
	return p;
}

void initList(List *pList) {
	// pList->ptr = malloc(sizeof(Node));
	/* pList->ptr->data = ???;		// dummy*/
	// pList->ptr->next = NULL;
	pList->ptr = createNode(-1, NULL);		// dummy create, -1은 의미 없는 인자라는 뜻.
}

void cleanupList(List *pList) {
	Node *p = pList->ptr;
	while(p ) {
		Node *tmp = p;
		p = p->next;
		
		free(tmp);
	}
}

void printList(const List *pList) {
	Node *p = pList->ptr->next;
	printf("[");
	while(p/* != NULL */) {
		printf("%d", p->data);
		printf((p->next/* != NULL */) ? ", " : "");
		p = p->next;
	}
	printf("]\n");
}

void insertFirstNode(List *pList, int data) {
	// Node *p = malloc(sizeof(Node));
	// p->data = data;
	// p->next = pList->ptr->next;
	// pList->ptr->next = p;				// pList->ptr는 dummy Node
	pList->ptr->next = createNode(data, pList->ptr->next);
}

void insertNode(List *pList, int prevData, int data) {
	Node *p = pList->ptr->next;
	while(p/* != NULL */) {
		if(p->data == prevData) {
			break;
		}
		p = p->next;
	}
	if(p/* != NULL */) {
		// Node *p2 = malloc(sizeof(Node));
		// p2->data = data;
		// p2->next = p->next;
		// p->next = p2;
		p->next = createNode(data, p->next);
	} else {
		// 값 삽입 안하는 걸로...
	}
}

void deleteNode(List *pList, int data) {
	Node *p1 = pList->ptr->next;
	Node *p2 = pList->ptr;
	
	while(p1/* != NULL */) {
		if(p1->data == data) {
			break;
		}
		p1 = p1->next;
		p2 = p2->next;
	}
	
	if(p1/* != NULL */) {
		p2->next = p1->next;
		free(p1);		
	} else {
		// 
	}
}
