#ifndef QUEUE_H
#define QUEUE_H

/*
typedef struct {
	int *pArr;
	int size;
	int front;
	int rear;
} Queue;
*/

class Queue {
private:
	int *pArr;
	int size;
	int front;
	int rear;
	
public:
	// void initQueue(Queue *pq, int size);
	// void cleanupQueue(Queue *pq);
	Queue(int size);
	~Queue();

	// void push(Queue *pq, int data);
	// int pop(Queue *pq);
	void push(int data);
	int pop();
};

/*
void initQueue(Queue *pq, int size);
void cleanupQueue(Queue *pq);

void push(Queue *pq, int data);
int pop(Queue *pq);
*/
#endif
