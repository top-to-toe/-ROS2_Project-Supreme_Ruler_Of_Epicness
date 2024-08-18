// #include <stdio.h>
#include <iostream>
#include "queue.h"

using std::cout;
using std::endl;

int main() {
	// Queue q1, q2;
	// initQueue(&q1, 10);
	// initQueue(&q2, 100);
	// q1.Queue(10);
	// q2.Queue(100);
	Queue q1(10), q2(100);
	
	// push(&q1, 100);
	// push(&q1, 200);
	// push(&q1, 300);
	q1.push(100);
	q1.push(200);
	q1.push(300);
	
	// printf("q1 1st pop(): %d\n", pop(&q1));
	// printf("q1 2nd pop(): %d\n", pop(&q1));
	// printf("q1 3rd pop(): %d\n", pop(&q1));
	cout << "q1 1st pop(): " << q1.pop() << endl;
	cout << "q1 2nd pop(): " << q1.pop() << endl;
	cout << "q1 3rd pop(): " << q1.pop() << endl;
	
	cout << endl;
	
	// push(&q2, 900);
	// push(&q2, 800);
	// push(&q2, 700);
	q2.push(900);
	q2.push(800);
	q2.push(700);
	
	// printf("q2 1st pop(): %d\n", pop(&q2));
	// printf("q2 2nd pop(): %d\n", pop(&q2));
	// printf("q2 3rd pop(): %d\n", pop(&q2));
	cout << "q2 1st pop(): " << q2.pop() << endl;
	cout << "q2 2nd pop(): " << q2.pop() << endl;
	cout << "q2 3rd pop(): " << q2.pop() << endl;
	
	// cleanupQueue(&q1);
	// cleanupQueue(&q2);
	
	return 0;
}
