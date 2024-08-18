#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "list.h"

void initList(List *pList, int eleSize) {
	/* create dummy node(더미 노드 생성) */
	pList->ptr = malloc(sizeof(Node) /* + eleSize*/);	// pList->ptr는 dummy Node
	// pList->ptr->data = ???;									// create dummy node
	pList->ptr->next = NULL;
	pList->eleSize = eleSize;
}

void cleanupList(List *pList) {
	Node *p = pList->ptr;
	while(p ) {
		Node *tmp = p;
		p = p->next;
		
		free(tmp);
	}
}

void printList(const List *pList, void (*print)(const void *)) {
	Node *p = pList->ptr->next;
	printf("[");
	while(p/* != NULL */) {
		// printf("%d", p->data);
		// printf("%d", p + 1);
		(*print)(p + 1);
		printf((p->next/* != NULL */) ? ", " : "");
		p = p->next;
	}
	printf("]\n");
}

void insertFirstNode(List *pList, const void *pData) {
	// Node *p = malloc(sizeof(Node));
	Node *p = malloc(sizeof(Node) + pList->eleSize);
	// p->pData = pData;
	memcpy(p + 1, pData, pList->eleSize);
	p->next = pList->ptr->next;
	pList->ptr->next = p;				// pList->ptr는 dummy Node
}

void insertNode(List *pList, const void *pPrevData, const void *pData) {
	Node *p = pList->ptr->next;
	while(p/* != NULL */) {
		// if(p->data == prevData) {
		if(memcmp(p + 1, pPrevData, pList->eleSize) == 0) {
			break;
		}
		p = p->next;
	}
	if(p/* != NULL */) {
		// Node *p2 = malloc(sizeof(Node));
		Node *p2 = malloc(sizeof(Node) + pList->eleSize);
		// p2->data = data;
		memcpy(p2 + 1, pData, pList->eleSize);
		p2->next = p->next;
		p->next = p2;
	} else {
		// 값 삽입 안하는 걸로...
	}
}

void deleteNode(List *pList, const void *pData) {
	Node *p1 = pList->ptr->next;
	Node *p2 = pList->ptr;
	
	while(p1/* != NULL */) {
		// if(p1->data == data) {
		if(memcmp(p1 + 1, pData, pList->eleSize) == 0) {
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
