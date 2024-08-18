#ifndef STACK_H
#define STACK_H

/* typedef */ /* struct */ class Stack {
private:
	int *pArr;
	int size;
	int tos;

public:
	// void initStack(/* Stack *ps, */int size);
	Stack(int size);										// constructor
	// void cleanupStack(/* Stack *ps*/);
	~Stack();												// destructor

	void push(/* Stack *ps, */int data);
	int pop(/* Stack *ps, */);
}/* Stack */;

/*
void initStack(Stack *ps, int size);
void cleanupStack(Stack *ps);

void push(Stack *ps, int data);
int pop(Stack *ps);
*/

#endif
