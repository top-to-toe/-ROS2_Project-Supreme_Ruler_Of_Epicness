/* Generic Version */

#ifndef LIST_H
#define LIST_H

typedef struct node {
	// int data;
	struct node *next;
} Node;

typedef struct {
	Node *ptr;
	int eleSize;
} List;

void initList(List *pList, int eleSize);
void cleanupList(List *pList);

void printList(const List *pList, void (*print)(const void *));		// 임의의 인자가 받아질수 있도록...void *사용

void insertFirstNode(List *pList, const void *pData);
void insertNode(List *pList, const void *pPrevData, const void *pData);
void deleteNode(List *pList, const void *pData);

#endif
