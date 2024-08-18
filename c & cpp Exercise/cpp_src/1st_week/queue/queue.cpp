// #include <stdlib.h>
// #include <assert.h>
#include <cstdlib>
#include <cassert>
#include "queue.h"

/*
void initQueue(Queue *pq, int size) {
	// pq->pArr = malloc(sizeof(int) * size);
	pq->pArr = (int *)malloc(sizeof(int) * size);
	assert(pq->pArr);
	
	pq->size = size;
	pq->front = 0;
	pq->rear = 0;
}
*/

Queue::Queue(int size) {
	this->pArr = new int[size];
	assert(this->pArr);
	
	this->size = size;
	this->front = 0;
	this->rear = 0;
}

/*
void cleanupQueue(Queue *pq) {
	free(pq->pArr);
}
*/

Queue::~Queue() {
	delete [] this->pArr;
}

/*
void push(Queue *pq, int data) {
	assert(pq->rear != pq->size);
	
	pq->pArr[pq->rear] = data;
	++pq->rear;
}
*/

void Queue::push(int data) {
	assert(this->rear != this->size);
	
	this->pArr[this->rear] = data;
	++this->rear;
}

/*
int pop(Queue *pq) {
	assert(pq->rear != pq->front);
	
	int i = pq->front;
	++pq->front;
	
	return pq->pArr[i];
}
*/

int Queue::pop() {
	assert(this->rear != this->front);
	
	int i = this->front;
	++this->front;
	
	return this->pArr[i];
}
