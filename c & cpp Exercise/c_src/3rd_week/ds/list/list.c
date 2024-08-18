#include <stdio.h>
#include <stdlib.h>

typedef struct node {
	int data;
	struct node *next;
} Node;

int main(void) {
	Node *ptr;
	
	ptr= malloc(sizeof(Node));
	ptr->data = 1;
	ptr->next = malloc(sizeof(Node));
	ptr->next->data = 2;
	ptr->next->next = malloc(sizeof(Node));
	ptr->next->next->data = 4;
	ptr->next->next->next = NULL;
	
	// 3 노드 삽입
	{
		Node *p;
		p = malloc(sizeof(Node));
		p->data = 3;
		p->next = ptr->next->next;
		ptr->next->next = p;
	}
	
	// 2 노드 삭제
	{
		Node *p;
		p = ptr->next;
		ptr->next = p->next;
		free(p);
	}
	
	// list 출력
	{
		Node *p;
		p = ptr;
		while(p/* != NULL*/) {
			printf("%d", p->data);
			printf((p->next/* != NULL */) ? ", " : "");
			p = p->next;
		}
		printf("\n");
	}	
	return 0;
}
